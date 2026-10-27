#!/usr/bin/env python3
"""Find the smallest version the next release may have.

Usage: scripts/semver_check.py [<version>]

The version may be a tag like v1.2.3.

cargo-semver-checks compares every publishable library crate with its last
release. That is the crates.io version, or the latest v* tag for a crate that
is not on crates.io. The next version counts up from the newest release of
the repository, on crates.io or as a tag.

Without an argument the script prints that version on stdout. With one it
checks that the given version is at least that, and fails otherwise. Reports
go to stderr.

cargo-semver-checks sees the Rust API only. Wire formats, configuration files,
CLI flags and feature defaults need a human. Give a higher version for those.

The file is the same in every SeaKit tool.
"""

import json
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

LEVELS = ["patch", "minor", "major"]
VERSION = re.compile(r"^(\d+)\.(\d+)\.(\d+)(?:-([0-9A-Za-z.-]+))?(?:\+[0-9A-Za-z.-]+)?$")


def log(message=""):
    print(message, file=sys.stderr)


def parse(version):
    match = VERSION.match(version)
    if not match:
        return None
    major, minor, patch, pre = match.groups()
    # A pre-release sorts before its release.
    return (int(major), int(minor), int(patch), pre is None, pre or "")


def bump(version, level):
    """The next version for a change of `level`. Cargo reads 0.y.z with y as
    the breaking part, so before 1.0 a breaking change raises the minor version
    and everything else the patch version. Every 0.0.z release counts as
    breaking already, so 0.0.z only ever raises the patch version."""
    major, minor, patch = parse(version)[:3]
    if major == 0 and minor == 0:
        return f"0.0.{patch + 1}"
    if major == 0:
        level = {"major": "minor", "minor": "patch", "patch": "patch"}[level]
    if level == "major":
        return f"{major + 1}.0.0"
    if level == "minor":
        return f"{major}.{minor + 1}.0"
    return f"{major}.{minor}.{patch + 1}"


def run(*args, cwd=None):
    return subprocess.run(args, cwd=cwd, capture_output=True, text=True)


def crates_io_version(name, below):
    """The newest version of `name` on crates.io, ignoring yanked versions and
    those at or above `below`."""
    request = urllib.request.Request(
        f"https://crates.io/api/v1/crates/{name}/versions",
        headers={"User-Agent": "seakit-semver-check (codeberg.org/stelzo)"},
    )
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            versions = json.load(response)["versions"]
    except urllib.error.HTTPError as error:
        if error.code == 404:
            return None
        raise
    numbers = [v["num"] for v in versions if not v["yanked"] and parse(v["num"])]
    if below:
        numbers = [n for n in numbers if parse(n) < parse(below)]
    return max(numbers, key=parse, default=None)


def latest_tag(root, below):
    """The newest v* tag, ignoring tags at or above `below` so a release
    never compares with itself."""
    tags = run("git", "tag", "--list", "v[0-9]*", cwd=root).stdout.split()
    versions = [t[1:] for t in tags if parse(t[1:])]
    if below:
        versions = [v for v in versions if parse(v) < parse(below)]
    return max(versions, key=parse, default=None)


def library_crates(root):
    metadata = json.loads(
        run("cargo", "metadata", "--no-deps", "--format-version", "1", cwd=root).stdout
    )
    for package in metadata["packages"]:
        if package["publish"] == []:
            continue
        if any(kind in ("lib", "rlib") for t in package["targets"] for kind in t["kind"]):
            manifest = Path(package["manifest_path"]).relative_to(root)
            yield package["name"], manifest.as_posix()


def check(root, name, baseline):
    """Run cargo-semver-checks for one crate and return the level it requires.

    It checks with all features first. Some features build only with system
    software, like a sourced ROS installation. If that build fails, the check
    falls back to the crate's default features and says so."""
    for features in ([], ["--default-features"]):
        output = run(
            "cargo", "semver-checks", "check-release", "--package", name,
            "--release-type", "patch", *features, *baseline, cwd=root,
        )
        text = output.stdout + output.stderr
        summary = re.search(r"Summary\s+(.*)", text)
        if summary:
            break
        if not features:
            log("    all features failed to build, checking default features only")
    if not summary:
        missing = re.search(r"failed to select a version for the requirement `([^`]+)`", text)
        if missing:
            sys.exit(
                f"{name} needs {missing.group(1)}, which is not on crates.io. "
                "Publish that first. cargo-semver-checks builds outside the workspace, "
                "so [patch] entries do not apply."
            )
        log(text)
        sys.exit(f"cargo-semver-checks failed for {name}")
    summary = summary.group(1)
    for line in text.splitlines():
        if line.startswith("--- failure"):
            log(f"    {line.removeprefix('--- failure ').rstrip(' -')}")
    if "new major version" in summary:
        return "major"
    if "new minor version" in summary:
        return "minor"
    return "patch"


def main():
    wanted = sys.argv[1].removeprefix("v") if len(sys.argv) > 1 else None
    if wanted and not parse(wanted):
        sys.exit(f"not a semver version: {wanted}")
    if not shutil.which("cargo-semver-checks"):
        sys.exit("cargo-semver-checks is missing. Install it with: cargo install --locked cargo-semver-checks")

    root = Path(run("git", "rev-parse", "--show-toplevel").stdout.strip())
    tag = latest_tag(root, wanted)
    releases = [tag] if tag else []
    level = "patch"

    for name, manifest in library_crates(root):
        published = crates_io_version(name, wanted)
        if published:
            baseline = ["--baseline-version", published]
            source = f"crates.io {published}"
            releases.append(published)
        elif tag and run("git", "cat-file", "-e", f"v{tag}:{manifest}", cwd=root).returncode == 0:
            baseline = ["--baseline-rev", f"v{tag}"]
            source = f"tag v{tag}"
        else:
            log(f"{name}: no earlier release, skipped")
            continue
        log(f"{name}: checking against {source}")
        required = check(root, name, baseline)
        log(f"{name}: {required}")
        level = max(level, required, key=LEVELS.index)

    if not releases:
        if wanted:
            log("No earlier release, so any version will do.")
            return
        sys.exit("No earlier release to count from. Give the version as an argument.")

    base = max(releases, key=parse)
    minimum = bump(base, level)
    log("")
    log(f"Latest release {base}, the API change is {level} level, the next version is at least {minimum}.")

    if not wanted:
        print(minimum)
    elif parse(wanted) < parse(minimum):
        sys.exit(f"{wanted} is too low. The next version must be at least {minimum}.")


if __name__ == "__main__":
    main()
