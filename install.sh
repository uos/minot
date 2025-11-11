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
    -r, --ros-distro DISTRO   ROS distribution (humble, jazzy) for ROS-specific builds
    -b, --build               Force build from source (skip binary download)
    -y, --yes                 Assume yes to all prompts

${BOLD}EMBED COMPONENTS:${NC}
    coord         Embedded coordinator (default)
    ros1          ROS1 publisher (native, no system deps)
    ros2          ROS2 publisher (native, no system deps)
    ros2-c        ROS2 publisher (C API, needs sourced ROS2)
    ratpub        Ratpub publisher
    ros           Both ROS1 and ROS2-C (needs both sourced)
    ros-native    Both ROS1 and ROS2 native (no system deps)

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

    ${CYAN}# Install to custom directory${NC}
    ./install.sh --dir /usr/local/bin

    ${CYAN}# Non-interactive mode${NC}
    ./install.sh --yes --build --embed ros-native

For more information, visit: ${BLUE}https://uos.github.io/minot/installation.html${NC}

EOF
}

# Convert embed components to cargo features
embed_to_features() {
    EMBED_LIST="$1"
    RESULT=""
    
    # Split by comma and process each component
    OLD_IFS="$IFS"
    IFS=","
    for component in $EMBED_LIST; do
        component=$(echo "$component" | tr -d ' ')
        case "$component" in
            coord)
                RESULT="${RESULT},embed-coord"
                ;;
            ros1)
                RESULT="${RESULT},embed-ros1-native"
                ;;
            ros2)
                RESULT="${RESULT},embed-ros2-native"
                ;;
            ros2-c)
                RESULT="${RESULT},embed-ros2-c"
                ;;
            ratpub)
                RESULT="${RESULT},embed-ratpub"
                ;;
            ros)
                RESULT="${RESULT},embed-ros"
                ;;
            ros-native)
                RESULT="${RESULT},embed-ros-native"
                ;;
            *)
                warn "Unknown embed component: $component (will be ignored)"
                ;;
        esac
    done
    IFS="$OLD_IFS"
    
    # Remove leading comma
    RESULT=$(echo "$RESULT" | sed 's/^,//')
    echo "$RESULT"
}

# Parse command line arguments
parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            -h|--help)
                usage
                exit 0
                ;;
            -v|--version)
                VERSION="$2"
                shift 2
                ;;
            -d|--dir)
                INSTALL_DIR="$2"
                shift 2
                ;;
            -e|--embed)
                EMBED_COMPONENTS="$2"
                shift 2
                ;;
            -f|--features)
                FEATURES="$2"
                shift 2
                ;;
            -r|--ros-distro)
                ROS_DISTRO="$2"
                shift 2
                ;;
            -b|--build)
                FORCE_BUILD=1
                shift
                ;;
            -y|--yes)
                YES_TO_ALL=1
                shift
                ;;
            *)
                error "Unknown option: $1\nRun 'install.sh --help' for usage information."
                ;;
        esac
    done
}

# Detect OS
detect_os() {
    OS=$(uname -s)
    case "$OS" in
        Linux*)
            echo "linux"
            ;;
        Darwin*)
            echo "macos"
            ;;
        *)
            error "Unsupported operating system: $OS"
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
            error "Unsupported architecture: $ARCH"
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
                x86_66)
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

# Get latest release version from GitHub
get_latest_version() {
    info "Fetching latest release version..."
    
    if command -v curl >/dev/null 2>&1; then
        # Use a cautious curl invocation; if it fails, fall back to build-from-source
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
            read -r response
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
        # Default to embed-coord if no features specified
        BUILD_FEATURES="embed-coord"
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
                    read -r resp
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

    # Prepare cargo install command
    CARGO_CMD="cargo install --git https://github.com/${REPO}.git minot --locked --features $BUILD_FEATURES"

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
    read -r resp
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
main() {
    parse_args "$@"
    
    printf "\n"
    info "Minot Installation Script"
    printf "\n"

    # If an existing installation is present, offer to uninstall (or auto-uninstall with -y)
    prompt_uninstall_existing
    
    # Detect system
    OS_NAME=$(detect_os)
    ARCH_NAME=$(detect_arch)
    
    # Prefer GNU targets for ROS builds
    PREFER_GNU=0
    if [ -n "$ROS_DISTRO" ]; then
        PREFER_GNU=1
    fi
    
    TARGET=$(get_target_triple "$OS_NAME" "$ARCH_NAME" "$PREFER_GNU")
    
    info "Detected system: ${CYAN}$OS_NAME${NC} (${CYAN}$ARCH_NAME${NC})"
    info "Target triple: ${CYAN}$TARGET${NC}"
    
    # Try to download prebuilt binary unless forced to build
    if [ $FORCE_BUILD -eq 0 ]; then
    
        if [ -z "$VERSION" ]; then
            if ! get_latest_version; then
                # If fetching version fails, fall back to building from source
                FORCE_BUILD=1
            fi
        else
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
                read -r response
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