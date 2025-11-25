#!/bin/sh
# Minot Installation Script
# This script installs Minot binaries from GitHub releases or builds from source

set -e

# Default values
REPO="uos/minot"
INSTALL_DIR="${HOME}/.local/bin"
GITHUB_API="https://api.github.com/repos/${REPO}"
GITHUB_RELEASE="https://github.com/${REPO}/releases/download"
VERSION=""
FEATURES=""
EMBED_COMPONENTS=""
ROS_DISTRO=""
FORCE_BUILD=0
YES_TO_ALL=0
NO_DEFAULT_EMBED=0

# Color output (always enabled for better readability)
RED=$(printf '\033[0;31m')
GREEN=$(printf '\033[0;32m')
YELLOW=$(printf '\033[1;33m')
BLUE=$(printf '\033[0;34m')
CYAN=$(printf '\033[0;36m')
BOLD=$(printf '\033[1m')
NC=$(printf '\033[0m') # No Color

info() {
    printf "${BLUE}${BOLD}==>${NC} ${BOLD}%s${NC}\n" "$1"
}

success() {
    printf "${GREEN}${BOLD}✓${NC} ${GREEN}%s${NC}\n" "$1"
}

warn() {
    printf "${YELLOW}${BOLD}⚠${NC} ${YELLOW}%s${NC}\n" "$1"
}

error() {
    printf "${RED}${BOLD}✗${NC} ${RED}%s${NC}\n" "$1" >&2
    exit 1
}

prompt() {
    printf "${CYAN}?${NC} %s " "$1"
}

# Prefer the current stdin if it's a terminal,
# otherwise try /dev/tty. Returns non-zero when no interactive input
# source is available.
safe_read() {
    # usage: safe_read varname
    VAR_NAME="$1"
    if [ -t 0 ]; then
        read -r "$VAR_NAME" || return 1
    elif [ -r /dev/tty ]; then
        read -r "$VAR_NAME" < /dev/tty 2>/dev/null || return 1
    else
        return 1
    fi
    return 0
}

# Print usage information
usage() {
    cat <<EOF
${BOLD}Minot Installation Script${NC}

${BOLD}USAGE:${NC}
    install.sh [OPTIONS]

${BOLD}OPTIONS:${NC}
    -h, --help                Show this help message
    -v, --version VERSION     Install specific version (default: latest)
    -d, --dir DIR             Installation directory (default: ~/.local/bin)
    -e, --embed COMPONENTS    Embed components (comma-separated: coord, ros1, ros2, ros2-c, ratpub, ros, ros-native)
    -f, --features FEATURES   Advanced: Raw cargo features (comma-separated)
    -r, --ros-distro DISTRO   ROS2 C API binding to embed when building (jazzy, humble). Requires ROS2 sourced.
    -b, --build               Force build from source (skip binary download)
    -y, --yes                 Assume yes to all prompts
    -n, --no-default-embed    Do not enable the default 'coord' embed when building

${BOLD}EMBED COMPONENTS:${NC}
    coord         Embedded coordinator (default)
    ratpub        Ratpub publisher
    ros           Both ROS1 and ROS2-C (needs ROS2 sourced)
    ros1          ROS1 publisher (native, no system dependencies)
    ros2          ROS2 publisher with RustDDS (native, no system dependencies)
    ros-native    Both ROS1 and ROS2 native (no system dependencies)

${BOLD}EXAMPLES:${NC}
    ${CYAN}# Install latest version from prebuilt binary${NC}
    ./install.sh

    ${CYAN}# Single-line installation${NC}
    curl -sSf https://raw.githubusercontent.com/uos/minot/main/install.sh | sh

    ${CYAN}# Install with ROS2 Jazzy support${NC}
    ./install.sh --ros-distro jazzy

    ${CYAN}# Build with embedded ROS2 native support${NC}
    ./install.sh --build --embed ros2

    ${CYAN}# Build with multiple embedded components${NC}
    ./install.sh --build --embed coord,ros1,ros2

For more information, visit: ${BLUE}https://uos.github.io/minot/installation.html${NC}

EOF
}

get_latest_version() {
        info "Fetching latest release version..."
        if command -v curl >/dev/null 2>&1; then
            VERSION=$(curl -sSf "${GITHUB_API}/releases/latest" 2>/dev/null | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/' || true)
        elif command -v wget >/dev/null 2>&1; then
            VERSION=$(wget -qO- "${GITHUB_API}/releases/latest" 2>/dev/null | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/' || true)
        else
            warn "Neither curl nor wget found. Will fall back to building from source."
            return 1
        fi

        if [ -z "$VERSION" ]; then
            warn "Failed to fetch latest version from GitHub. Will fall back to building from source."
            return 1
        fi

        success "Latest version: ${CYAN}$VERSION${NC}"
        return 0
    }

detect_os() {
    # Use uname -s for reliable OS detection (works when script is piped)
    local uname_s
    uname_s=$(uname -s 2>/dev/null || true)

    case "$uname_s" in
        Linux*)
            echo "linux"
            ;;
        Darwin*)
            echo "macos"
            ;;
        CYGWIN*|MINGW*|MSYS*)
            echo "windows"
            ;;
        *)
            error "Unsupported operating system: ${uname_s:-unknown}"
            ;;
    esac
}

# Detect architecture
detect_arch() {
    ARCH=$(uname -m)
    case "$ARCH" in
        x86_64|amd64)
            echo "x86_64"
            ;;
        aarch64|arm64)
            echo "aarch64"
            ;;
        armv7l|armhf)
            echo "armv7"
            ;;
        *)
            warn "Unknown architecture: $ARCH; cannot determine a prebuilt target triple"
            # Return empty to indicate we couldn't detect a known architecture.
            echo ""
            ;;
    esac
}

# Map OS and architecture to Rust target triple
get_target_triple() {
    OS_NAME=$1
    ARCH_NAME=$2
    PREFER_GNU=$3
    
    case "$OS_NAME" in
        linux)
            case "$ARCH_NAME" in
                    x86_64)
                        # ROS builds use gnu, otherwise prefer musl for better portability
                        if [ "$PREFER_GNU" = "1" ]; then
                            echo "x86_64-unknown-linux-gnu"
                        else
                            echo "x86_64-unknown-linux-musl"
                        fi
                        ;;
                aarch64)
                    echo "aarch64-unknown-linux-gnu"
                    ;;
                armv7)
                    echo "armv7-unknown-linux-gnueabihf"
                    ;;
                *)
                    # Unknown architecture: cannot map to a prebuilt target triple.
                    # Return empty to indicate inability to determine a valid triple.
                    echo ""
                    return 1
                    ;;
            esac
            ;;
        macos)
            case "$ARCH_NAME" in
                x86_64)
                    echo "x86_64-apple-darwin"
                    ;;
                aarch64)
                    echo "aarch64-apple-darwin"
                    ;;
            esac
            ;;
    esac
}


detect_ros_from_env_and_prompt() {
    # If user already set --ros-distro, respect it
    if [ -n "$ROS_DISTRO" ]; then
        return 0
    fi

    # Prefer environment variable ROS_DISTRO if present
    local detected=""
    if [ -n "${ROS_DISTRO}" ]; then
        detected="${ROS_DISTRO}"
    fi

    # Also try commonly set ROS env var
    if [ -z "$detected" ] && [ -n "${ROS_DISTRO}" ]; then
        detected="${ROS_DISTRO}"
    fi

    # Try PATH detection as fallback: look for /opt/ros/<distro>/bin in PATH
    if [ -z "$detected" ] && echo ":${PATH}:" | grep -q '/opt/ros/'; then
        # extract distro name from first /opt/ros/<distro>/bin occurrence
        detected=$(echo ":${PATH}:" | sed -n 's/.*:\/opt\/ros\/\([^:]*\)\/bin:.*/\1/p' | sed -n '1p' || true)
    fi

    if [ -z "$detected" ]; then
        return 0
    fi

    # Normalize to lowercase
    detected=$(echo "$detected" | tr '[:upper:]' '[:lower:]')

    # Supported list (if not in this list, fall back to 'jazzy')
    case "$detected" in
        jazzy|humble|galactic|foxy|iron|rolling)
            ;; # supported
        *)
            warn "Detected ROS distro '$detected' is not in supported list; falling back to 'jazzy'."
            detected="jazzy"
            ;;
    esac

    # Ask the user whether to use this distro (unless -y / YES_TO_ALL)
    if [ $YES_TO_ALL -eq 1 ]; then
        ROS_DISTRO="$detected"
        info "Auto-enabled ROS distro: ${CYAN}$ROS_DISTRO${NC} (from environment)"
        return 0
    fi

    printf "\n"
    prompt "Detected ROS distro in your environment: '${detected}'. Install with this distro? [Y/n]"
    if safe_read resp; then
        case "$resp" in
            [nN][oO]|[nN])
                info "Not enabling ROS embed by default."
                ;;
            *)
                ROS_DISTRO="$detected"
                info "Enabled ROS distro: ${CYAN}$ROS_DISTRO${NC}"
                ;;
        esac
    else
        warn "Could not read response; not enabling ROS embed by default."
    fi
}

parse_semver() {
    # parse_semver "v0.1.0-rc.6" --> prints: major minor patch prerelease raw
    local s="$1"
    # strip leading v
    s=${s#v}
    # extract semver-like portion
    if echo "$s" | grep -Eq '^[0-9]+\.[0-9]+\.[0-9]+'; then
        local sem=$(echo "$s" | grep -Eo '^[0-9]+\.[0-9]+\.[0-9]+(-[^+]+)?')
        local major=$(echo "$sem" | cut -d. -f1)
        local minor=$(echo "$sem" | cut -d. -f2)
        local patch_with=$(echo "$sem" | cut -d. -f3)
        local patch=$(echo "$patch_with" | sed -E 's/[^0-9].*$//')
        local prerelease=""
        if echo "$sem" | grep -q '-'; then
            prerelease=$(echo "$sem" | sed -E 's/^[0-9]+\.[0-9]+\.[0-9]+-//')
        fi
        echo "$major $minor $patch $prerelease $sem"
        return 0
    fi
    return 1
}

compare_semver_parts() {
    # compare two triplets: returns 0 if a==b, 1 if a>b, 2 if a<b
    local a_major=$1 a_minor=$2 a_patch=$3
    local b_major=$4 b_minor=$5 b_patch=$6
    if [ "$a_major" -gt "$b_major" ]; then return 1; fi
    if [ "$a_major" -lt "$b_major" ]; then return 2; fi
    if [ "$a_minor" -gt "$b_minor" ]; then return 1; fi
    if [ "$a_minor" -lt "$b_minor" ]; then return 2; fi
    if [ "$a_patch" -gt "$b_patch" ]; then return 1; fi
    if [ "$a_patch" -lt "$b_patch" ]; then return 2; fi
    return 0
}

compat_upper_bound() {
    # Given major minor patch, print upper bound triplet (major,minor,patch)
    local major=$1 minor=$2 patch=$3
    if [ "$major" -eq 0 ]; then
        echo "$major $((minor + 1)) 0"
    else
        echo "$((major + 1)) 0 0"
    fi
}

is_in_range() {
    # is_in_range candidate >= required && candidate < upper
    local c1=$1 c2=$2 c3=$3
    local r1=$4 r2=$5 r3=$6
    local u1=$7 u2=$8 u3=$9

    # if candidate < required -> false
    compare_semver_parts "$c1" "$c2" "$c3" "$r1" "$r2" "$r3"
    local cmp_to_req=$?
    if [ $cmp_to_req -eq 2 ]; then
        return 1
    fi

    # if candidate >= upper -> false
    compare_semver_parts "$c1" "$c2" "$c3" "$u1" "$u2" "$u3"
    local cmp_to_upper=$?
    if [ $cmp_to_upper -ne 2 ]; then
        # cmp_to_upper == 0 means equal -> >= upper, cmp_to_upper ==1 means > upper
        return 1
    fi

    return 0
}
 

resolve_requested_version() {
    # If $VERSION is a prefix like '0.1', resolve it to the newest release
    # tag from GitHub that matches that major/minor. If VERSION is empty or
    # already a full tag/semver, leave it as-is.
    if [ -z "$VERSION" ]; then
        return 0
    fi

    # If user provided X.Y (prefix)
    if echo "$VERSION" | grep -Eq '^[0-9]+\.[0-9]+$'; then
        local orig="$VERSION"
        local want_major=$(echo "$VERSION" | cut -d. -f1)
        local want_minor=$(echo "$VERSION" | cut -d. -f2)
        local per_page=100
        local page=1
        local found_prerelease=""

        while :; do
            local releases_json=""
            if command -v curl >/dev/null 2>&1; then
                releases_json=$(curl -sSf "${GITHUB_API}/releases?per_page=${per_page}&page=${page}" 2>/dev/null || true)
            elif command -v wget >/dev/null 2>&1; then
                releases_json=$(wget -qO- "${GITHUB_API}/releases?per_page=${per_page}&page=${page}" 2>/dev/null || true)
            else
                warn "Neither curl nor wget found; cannot resolve version prefix '$VERSION'."
                return 1
            fi

            # If no JSON returned or empty page, stop
            if [ -z "$releases_json" ]; then
                break
            fi

            local any_tag=0
            for tag in $(printf "%s" "$releases_json" | grep '"tag_name"' | sed -E 's/.*"([^\"]+)".*/\1/' ); do
                any_tag=1
                t=${tag#v}
                parsed=$(parse_semver "$t" 2>/dev/null || true)
                if [ -z "$parsed" ]; then
                    continue
                fi
                IFS=' ' read c_major c_minor c_patch c_prerelease c_sem <<EOF
$parsed
EOF
                if [ "$c_major" -eq "$want_major" ] && [ "$c_minor" -eq "$want_minor" ]; then
                    if [ -z "$c_prerelease" ]; then
                        VERSION="$tag"
                        success "Resolved requested prefix '${CYAN}$orig${NC}' to ${CYAN}$VERSION${NC}"
                        return 0
                    else
                        # remember the first prerelease candidate in case no stable exists
                        if [ -z "$found_prerelease" ]; then
                            found_prerelease="$tag"
                        fi
                    fi
                fi
            done

            # If no tags on this page, stop paging
            if [ $any_tag -eq 0 ]; then
                break
            fi

            page=$((page + 1))
        done

        # If we didn't find a stable release but found a prerelease candidate, use it
        if [ -n "$found_prerelease" ]; then
            VERSION="$found_prerelease"
            success "Resolved requested prefix '${CYAN}$orig${NC}' to prerelease ${CYAN}$VERSION${NC}"
            return 0
        fi

        warn "No release found matching prefix '$orig'"
        return 1
    fi

    # Otherwise assume VERSION is a concrete tag or semver and accept it.
    return 0
}

# Check if binary exists for target
check_binary_exists() {
    TARGET=$1
    ARCHIVE_NAME=$2
    VERSION=$3
    
    URL="${GITHUB_RELEASE}/${VERSION}/${ARCHIVE_NAME}.tar.gz"
    
    if command -v curl >/dev/null 2>&1; then
        if curl -sSfI "$URL" >/dev/null 2>&1; then
            return 0
        fi
    elif command -v wget >/dev/null 2>&1; then
        if wget -q --spider "$URL" 2>/dev/null; then
            return 0
        fi
    fi
    
    return 1
}

# Download and install binary
download_and_install() {
    TARGET=$1
    ARCHIVE_NAME=$2
    VERSION=$3
    
    ARCHIVE_FILE="${ARCHIVE_NAME}.tar.gz"
    URL="${GITHUB_RELEASE}/${VERSION}/${ARCHIVE_FILE}"
    
    info "Downloading ${ARCHIVE_FILE}..."
    
    TMP_DIR=$(mktemp -d)
    trap "rm -rf '$TMP_DIR'" EXIT

    cd "$TMP_DIR" || { warn "Failed to enter temp dir; falling back to build"; return 1; }

    if command -v curl >/dev/null 2>&1; then
        if ! curl -sSfL "$URL" -o "$ARCHIVE_FILE"; then
            warn "Failed to download binary archive from GitHub: $URL"
            return 1
        fi
    elif command -v wget >/dev/null 2>&1; then
        if ! wget -q "$URL" -O "$ARCHIVE_FILE"; then
            warn "Failed to download binary archive from GitHub: $URL"
            return 1
        fi
    else
        warn "Neither curl nor wget found; cannot download binaries"
        return 1
    fi

    info "Extracting archive..."
    if ! tar xzf "$ARCHIVE_FILE"; then
        warn "Failed to extract archive: $ARCHIVE_FILE"
        return 1
    fi

    # Create install directory if it doesn't exist
    mkdir -p "$INSTALL_DIR"

    info "Installing binaries to $INSTALL_DIR..."

    INSTALLED_ANY=0
    # Install all binaries from the archive
    if [ -d "$ARCHIVE_NAME" ]; then
        for binary in "$ARCHIVE_NAME"/*; do
            if [ -f "$binary" ] && [ -x "$binary" ]; then
                BINARY_NAME=$(basename "$binary")
                cp "$binary" "$INSTALL_DIR/$BINARY_NAME"
                chmod +x "$INSTALL_DIR/$BINARY_NAME"
                success "Installed: $BINARY_NAME"
                INSTALLED_ANY=1
            fi
        done
    fi

    # Install library files if present
    for lib_file in "$ARCHIVE_NAME"/*.a "$ARCHIVE_NAME"/*.so "$ARCHIVE_NAME"/*.dylib "$ARCHIVE_NAME"/*.h; do
        if [ -f "$lib_file" ]; then
            FILENAME=$(basename "$lib_file")
            cp "$lib_file" "$INSTALL_DIR/$FILENAME"
            success "Installed library: $FILENAME"
            INSTALLED_ANY=1
        fi
    done

    if [ $INSTALLED_ANY -eq 0 ]; then
        warn "Archive did not contain any installable files; falling back to building from source"
        return 1
    fi

    cd - >/dev/null || true
    return 0
}

# Check if Rust is installed
check_rust() {
    if ! command -v cargo >/dev/null 2>&1; then
        warn "Rust is not installed on your system."
        printf "\n${BOLD}To build Minot from source, you need to install Rust.${NC}\n"
        printf "Visit: ${BLUE}https://rustup.rs/${NC}\n\n"
        printf "Quick install command:\n"
        printf "  ${GREEN}curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh${NC}\n\n"
        
        if [ $YES_TO_ALL -eq 0 ]; then
            prompt "Would you like to install Rust now? [Y/n]"
            if ! safe_read response; then
                error "Could not read input from terminal. Re-run interactively or use --yes to accept defaults."
            fi
            case "$response" in
                [nN][oO]|[nN])
                    exit 1
                    ;;
                *)
                    info "Installing Rust..."
                    if command -v curl >/dev/null 2>&1; then
                        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y || error "Failed to install Rust"
                        . "$HOME/.cargo/env"
                        success "Rust installed successfully!"
                    else
                        error "curl is required to install Rust. Please install curl or Rust manually."
                    fi
                    ;;
            esac
        else
            error "Rust is required to build from source. Install it from https://rustup.rs/"
        fi
    fi
    return 0
}

# Build from source
build_from_source() {
    info "Building Minot from source..."

    if ! check_rust; then
        error "Rust is required to build from source"
    fi

    # Convert embed components to features if specified
    if [ -n "$EMBED_COMPONENTS" ]; then
        EMBED_FEATURES=$(embed_to_features "$EMBED_COMPONENTS")
        if [ -n "$FEATURES" ]; then
            FEATURES="${FEATURES},${EMBED_FEATURES}"
        else
            FEATURES="$EMBED_FEATURES"
        fi
    fi

    # Determine features to build with
    BUILD_FEATURES="$FEATURES"
    if [ -z "$BUILD_FEATURES" ]; then
        # Default to embed-coord if no features specified, unless user opted out
        if [ "$NO_DEFAULT_EMBED" -eq 0 ]; then
            BUILD_FEATURES="embed-coord"
        else
            BUILD_FEATURES=""
        fi
    fi

    if [ -n "$ROS_DISTRO" ]; then
        ROS_DISTRO_LC=$(echo "$ROS_DISTRO" | tr '[:upper:]' '[:lower:]')
        if [ "$ROS_DISTRO_LC" = "jazzy" ]; then
            ROS2_C_FEATURE="embed-ros2-c"
        else
            ROS2_C_FEATURE="embed-ros2-c-${ROS_DISTRO_LC}"
        fi
        case ",$BUILD_FEATURES," in
            *,${ROS2_C_FEATURE},*|*,embed-ros,*|*,embed-ros2-c,*)
                # already present, a combined 'embed-ros' feature exists, or generic embed-ros2-c present; do nothing
                ;;
            *)
                if [ -n "$BUILD_FEATURES" ]; then
                    BUILD_FEATURES="${BUILD_FEATURES},${ROS2_C_FEATURE}"
                else
                    BUILD_FEATURES="${ROS2_C_FEATURE}"
                fi
                ;;
        esac
    fi

    info "Building with features: ${CYAN}$BUILD_FEATURES${NC}"

    # If a ROS distro was requested, only require that ROS is sourced into the environment.
    # We can't reliably detect every install location, so check PATH for a sourced ROS
    # (e.g. /opt/ros/<distro>/bin) or a ros2 binary. If not sourced, offer to source any
    # setup script we can find under /opt/ros/*.
    USE_BASH_SOURCE=0
    ROS_SETUP=""
    if [ -n "$ROS_DISTRO" ]; then
        # Check for any /opt/ros/*/bin entry in PATH (this indicates ROS was sourced)
        if echo ":${PATH}:" | grep -Eq '/opt/ros/[^:]+/bin'; then
            info "ROS appears to be sourced (PATH contains /opt/ros/*/bin)."
        elif command -v ros2 >/dev/null 2>&1; then
            info "ROS appears to be available (found 'ros2' in PATH)."
        else
            # Not obviously sourced. Try to locate any setup script under /opt/ros
            FOUND_SETUP=""
            for p in /opt/ros/*/setup.bash /opt/ros/*/setup.sh /opt/ros/*/setup.zsh; do
                if [ -f "$p" ]; then
                    FOUND_SETUP="$p"
                    break
                fi
            done

            if [ -n "$FOUND_SETUP" ]; then
                ROS_SETUP="$FOUND_SETUP"
                if [ $YES_TO_ALL -eq 1 ]; then
                    info "Auto-sourcing ${ROS_SETUP} for the build (--yes)."
                    USE_BASH_SOURCE=1
                else
                    printf "\n"
                    prompt "ROS does not appear to be sourced. Source ${ROS_SETUP} now for the build? [Y/n]"
                    if ! safe_read resp; then
                        error "Could not read input from terminal. Re-run interactively or set --yes to auto-enable ROS embed."
                    fi
                    case "$resp" in
                        [nN][oO]|[nN])
                            error "Please source ROS in your shell (or open a terminal with ROS sourced) and re-run this script."
                            ;;
                        *)
                            USE_BASH_SOURCE=1
                            ;;
                    esac
                fi
            else
                error "ROS does not appear to be sourced and no setup script was found under /opt/ros. Please source ROS in your shell or install ROS."
            fi
        fi
    fi

    # Create install directory if it doesn't exist
    mkdir -p "$INSTALL_DIR"

    # Prepare cargo install command. Only pass --features if non-empty.
    CARGO_CMD="cargo install --git https://github.com/${REPO}.git minot --locked"
    if [ -n "$BUILD_FEATURES" ]; then
        CARGO_CMD="$CARGO_CMD --features $BUILD_FEATURES"
    fi

    # Add version/tag if specified
    if [ -n "$VERSION" ]; then
        CARGO_CMD="$CARGO_CMD --tag $VERSION"
    fi

    # Set custom installation root if not using default cargo bin
    if [ "$INSTALL_DIR" != "${HOME}/.cargo/bin" ]; then
        # Calculate the root (parent of bin directory)
        INSTALL_ROOT=$(dirname "$INSTALL_DIR")
        CARGO_CMD="$CARGO_CMD --root $INSTALL_ROOT"
    fi

    printf "\n${BOLD}Build command:${NC} ${CYAN}%s${NC}\n\n" "$CARGO_CMD"

    # Run cargo install. If we need to source ROS, run the build in a bash subshell that sources the setup file first.
    if [ "$USE_BASH_SOURCE" -eq 1 ]; then
        if [ -n "$ROS_SETUP" ]; then
            if command -v bash >/dev/null 2>&1; then
                info "Running build in a bash subshell with ROS sourced: ${ROS_SETUP}"
                bash -lc ". \"${ROS_SETUP}\" && ${CARGO_CMD}" || error "Build failed"
            else
                warn "bash not found; attempting to source using sh. This may fail for some ROS setups."
                . "${ROS_SETUP}" && eval "$CARGO_CMD" || error "Build failed"
            fi
        else
            error "Internal error: requested to source ROS but no setup script found."
        fi
    else
        eval "$CARGO_CMD" || error "Build failed"
    fi

    printf "\n"
    success "Installed: minot"
    success "Installed: minot-coord"
}

# Check if installation directory is in PATH
check_path() {
    case ":${PATH}:" in
        *":${INSTALL_DIR}:"*)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

# Add directory to PATH
suggest_path_update() {
    if ! check_path; then
        warn "Installation directory is not in your PATH: $INSTALL_DIR"
        printf "\nTo use Minot, add the following to your shell configuration file:\n"
        
        # Detect shell
        SHELL_NAME=$(basename "$SHELL")
        case "$SHELL_NAME" in
            bash)
                CONFIG_FILE="~/.bashrc"
                ;;
            zsh)
                CONFIG_FILE="~/.zshrc"
                ;;
            fish)
                CONFIG_FILE="~/.config/fish/config.fish"
                printf "  ${GREEN}set -gx PATH \$PATH ${INSTALL_DIR}${NC}\n\n"
                return
                ;;
            *)
                CONFIG_FILE="~/.profile"
                ;;
        esac
        
        printf "  ${GREEN}export PATH=\"\$PATH:${INSTALL_DIR}\"${NC}\n"
        printf "\nAdd this to: ${BLUE}${CONFIG_FILE}${NC}\n\n"
    fi
}

# Check if minot is already installed (in PATH or in the selected install dir)
is_minot_installed() {
    # If a 'minot' executable is on PATH, verify it behaves like Minot
    if command -v minot >/dev/null 2>&1; then
        MINOT_PATH=$(command -v minot)
        # Try to read a version string; if it prints something non-empty, accept it.
        MINOT_VER=$("$MINOT_PATH" --version 2>/dev/null || true)
        if [ -n "$MINOT_VER" ]; then
            return 0
        else
            # Found an executable named 'minot' but it didn't identify itself
            warn "Found 'minot' at ${MINOT_PATH} but it did not return a version; ignoring this entry."
        fi
    fi

    # Also consider installed if binaries exist in the chosen install directory
    if [ -x "${INSTALL_DIR}/minot" ] || [ -x "${INSTALL_DIR}/minot-coord" ]; then
        return 0
    fi

    return 1
}

# Prompt and (optionally) uninstall existing installation
prompt_uninstall_existing() {
    if ! is_minot_installed; then
        return 0
    fi

    INSTALLED_VER=""
    if command -v minot >/dev/null 2>&1; then
        INSTALLED_VER=$(minot --version 2>/dev/null || true)
    fi

    if [ -n "$INSTALLED_VER" ]; then
        info "Detected existing Minot: ${CYAN}${INSTALLED_VER}${NC}"
    else
        info "Detected existing Minot installation"
    fi

    if [ $YES_TO_ALL -eq 1 ]; then
        info "Auto-uninstall enabled (-y): attempting to uninstall existing Minot..."
        if command -v minot >/dev/null 2>&1; then
            if minot uninstall >/dev/null 2>&1; then
                success "Previous Minot uninstalled"
                return 0
            fi
            error "Failed to run 'minot uninstall'. Please uninstall the existing Minot manually and re-run this script."
        else
            # No 'minot' command available; try removing binaries from INSTALL_DIR
            if [ -f "${INSTALL_DIR}/minot" ] || [ -f "${INSTALL_DIR}/minot-coord" ]; then
                rm -f "${INSTALL_DIR}/minot" "${INSTALL_DIR}/minot-coord" || error "Failed to remove existing binaries from ${INSTALL_DIR}. Please remove them manually."
                success "Removed existing binaries from ${INSTALL_DIR}"
                return 0
            fi
            error "Cannot run 'minot uninstall' and no binaries found in ${INSTALL_DIR}; please uninstall manually."
        fi
    fi

    # Interactive prompt
    printf "\n"
    prompt "A previous Minot installation was detected. Would you like me to uninstall it now? [Y/n]"
    if ! safe_read resp; then
        error "Could not read input from terminal. Re-run interactively or use --yes to auto-uninstall."
    fi
    case "$resp" in
        [nN][oO]|[nN])
            error "Please uninstall the existing Minot before proceeding, or re-run with --yes to auto-uninstall."
            ;;
        *)
            info "Attempting to uninstall existing Minot..."
            if command -v minot >/dev/null 2>&1; then
                if minot uninstall >/dev/null 2>&1; then
                    success "Previous Minot uninstalled"
                    return 0
                fi
                error "Failed to run 'minot uninstall'. Please uninstall manually and re-run this script."
            else
                if [ -f "${INSTALL_DIR}/minot" ] || [ -f "${INSTALL_DIR}/minot-coord" ]; then
                    rm -f "${INSTALL_DIR}/minot" "${INSTALL_DIR}/minot-coord" || error "Failed to remove existing binaries from ${INSTALL_DIR}. Please remove them manually."
                    success "Removed existing binaries from ${INSTALL_DIR}"
                    return 0
                fi
                error "No uninstaller available and no binaries found in ${INSTALL_DIR}; please uninstall manually."
            fi
            ;;
    esac
}

# Main installation logic
parse_args() {
    # Parse command line arguments. POSIX-compatible.
    while [ $# -gt 0 ]; do
        case "$1" in
            -h|--help)
                usage
                exit 0
                ;;
            -v|--version)
                if [ -n "$2" ] && [ "${2#-}" = "$2" ]; then
                    VERSION="$2"
                    shift
                else
                    error "Missing value for --version"
                fi
                ;;
            -d|--dir)
                if [ -n "$2" ] && [ "${2#-}" = "$2" ]; then
                    INSTALL_DIR="$2"
                    shift
                else
                    error "Missing value for --dir"
                fi
                ;;
            -e|--embed)
                if [ -n "$2" ] && [ "${2#-}" = "$2" ]; then
                    EMBED_COMPONENTS="$2"
                    shift
                else
                    error "Missing value for --embed"
                fi
                ;;
            -f|--features)
                if [ -n "$2" ] && [ "${2#-}" = "$2" ]; then
                    FEATURES="$2"
                    shift
                else
                    error "Missing value for --features"
                fi
                ;;
            -r|--ros-distro)
                if [ -n "$2" ] && [ "${2#-}" = "$2" ]; then
                    ROS_DISTRO="$2"
                    shift
                else
                    error "Missing value for --ros-distro"
                fi
                ;;
            -b|--build)
                FORCE_BUILD=1
                ;;
            -y|--yes)
                YES_TO_ALL=1
                ;;
            -n|--no-default-embed)
                NO_DEFAULT_EMBED=1
                ;;
            --)
                shift
                break
                ;;
            -*)
                warn "Unknown option: $1"
                ;;
            *)
                # positional arguments are ignored for now
                ;;
        esac
        shift
    done
}

main() {
    parse_args "$@"
    
    printf "\n"
    info "Minot Installation Script"
    printf "\n"

    # If an existing installation is present, offer to uninstall (or auto-uninstall with -y)
    prompt_uninstall_existing
    # Detect ROS environment and prompt to enable embed if found
    detect_ros_from_env_and_prompt
    
    # Detect system
    OS_NAME=$(detect_os)
    ARCH_NAME=$(detect_arch)
    
    # Prefer GNU targets for ROS builds
    PREFER_GNU=0
    if [ -n "$ROS_DISTRO" ]; then
        PREFER_GNU=1
    fi
    
    TARGET=$(get_target_triple "$OS_NAME" "$ARCH_NAME" "$PREFER_GNU" ) || true

    info "Detected system: ${CYAN}$OS_NAME${NC} (${CYAN}$ARCH_NAME${NC})"
    if [ -n "$TARGET" ]; then
        info "Target triple: ${CYAN}$TARGET${NC}"
    else
        warn "Could not determine a prebuilt target triple; will build from source"
        FORCE_BUILD=1
    fi
    
    # Try to download prebuilt binary unless forced to build
    if [ $FORCE_BUILD -eq 0 ]; then
    
        if [ -z "$VERSION" ]; then
            if ! get_latest_version; then
                # If fetching version fails, fall back to building from source
                FORCE_BUILD=1
            fi
        else
            # If user supplied a custom VERSION, try to resolve prefixes like '0.1'
            if ! resolve_requested_version; then
                error "Failed to resolve requested version: $VERSION"
            fi
            info "Installing version: ${CYAN}$VERSION${NC}"
        fi
        
        # Determine archive name based on ROS distro
        if [ -n "$ROS_DISTRO" ]; then
            ARCHIVE_NAME="minot-${ROS_DISTRO}-${TARGET}"
            info "Looking for ROS ${CYAN}${ROS_DISTRO}${NC} prebuilt binary..."
        else
            ARCHIVE_NAME="minot-${TARGET}"
            info "Looking for prebuilt binary..."
        fi
        
        if check_binary_exists "$TARGET" "$ARCHIVE_NAME" "$VERSION"; then
            success "Found prebuilt binary!"
            printf "\n"
            if ! download_and_install "$TARGET" "$ARCHIVE_NAME" "$VERSION"; then
                warn "Installing prebuilt binary failed — will fall back to building from source"
                FORCE_BUILD=1
            fi
        else
            warn "No prebuilt binary found for your system"
            
            # If ROS distro was specified but no binary found, inform user
            if [ -n "$ROS_DISTRO" ]; then
                info "ROS ${CYAN}${ROS_DISTRO}${NC} build not available for ${CYAN}${TARGET}${NC}"
            fi
            
            if [ $YES_TO_ALL -eq 0 ]; then
                printf "\n"
                prompt "Would you like to build from source? [Y/n]"
                if ! safe_read response; then
                    error "Could not read input from terminal. Installation cancelled."
                fi
                case "$response" in
                    [nN][oO]|[nN])
                        error "Installation cancelled"
                        ;;
                    *)
                        FORCE_BUILD=1
                        ;;
                esac
            else
                FORCE_BUILD=1
            fi
        fi
    fi
    
    # Build from source if necessary
    if [ $FORCE_BUILD -eq 1 ]; then
        printf "\n"
        
        if [ -n "$VERSION" ]; then
            info "Building from source, targeting version: ${CYAN}$VERSION${NC}"
        else
            info "Building from source (latest)"
        fi
        
        build_from_source
    fi
    
    printf "\n"
    success "Installation Complete!"
    
    # Check if binaries are accessible
    suggest_path_update
    
    printf "\n${BOLD}Next steps:${NC}\n"
    printf "  ${GREEN}1.${NC} Verify installation: ${CYAN}minot --help${NC}\n"
    printf "  ${GREEN}2.${NC} Read the docs: ${BLUE}https://uos.github.io/minot/${NC}\n\n"
    
    printf "${BOLD}To uninstall:${NC}\n"
    printf "  Run the built-in uninstaller (this is the recommended way):\n"
    printf "    ${CYAN}minot uninstall${NC}\n\n"
    printf "  ${BOLD}Note:${NC} This will remove the 'minot' and 'minot-coord' binaries.\n"
    
}

# Run main function
main "$@"
