#!/usr/bin/env python3
"""Synchronize workspace dependency versions in tracked Markdown examples."""

import re
import subprocess
import sys
import tomllib
from pathlib import Path


def update_examples(text, names, version):
    names = "|".join(re.escape(name) for name in sorted(names))
    pattern = re.compile(
        rf"^(\s*(?:{names})\s*=\s*(?:\{{[^\n}}]*?\bversion\s*=\s*)?)"
        r"([\"'])([^\"'\n]+)\2",
        re.MULTILINE,
    )
    return pattern.sub(lambda m: f"{m[1]}{m[2]}{version}{m[2]}", text)


def main():
    root = Path(__file__).resolve().parent.parent
    workspace = tomllib.loads((root / "Cargo.toml").read_text())["workspace"]
    names = set()
    for member in workspace["members"]:
        package = tomllib.loads((root / member / "Cargo.toml").read_text())["package"]
        if package.get("version") == {"workspace": True}:
            names.add(package["name"])
    files = subprocess.check_output(
        ["git", "ls-files", "-z", "--", "*.md", "*.mdx"], cwd=root
    ).decode().split("\0")
    for name in filter(None, files):
        path = root / name
        if not path.is_file():
            continue
        original = path.read_text()
        updated = update_examples(original, names, sys.argv[1])
        if updated != original:
            path.write_text(updated)


if __name__ == "__main__":
    main()
