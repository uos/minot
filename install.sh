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
ROS_DISTRO=""
FORCE_BUILD=0

# Color output
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    NC='\033[0m' # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    NC=''
fi

info() {
    printf "${BLUE}==>${NC} %s\n" "$1"
}

success() {
    printf "${GREEN}==>${NC} %s\n" "$1"
}

warn() {
    printf "${YELLOW}Warning:${NC} %s\n" "$1"
}

error() {
    printf "${RED}Error:${NC} %s\n" "$1" >&2
    exit 1
}

# Print usage information
usage() {
    cat <<EOF
Minot Installation Script

USAGE:
    install.sh [OPTIONS]

OPTIONS:
    -h, --help              Show this help message
    -v, --version VERSION   Install specific version (default: latest)
    -d, --dir DIR           Installation directory (default: ~/.local/bin)
    -f, --features FEATURES Cargo features for building from source (comma-separated)
    -r, --ros-distro DISTRO ROS distribution (humble, jazzy) for ROS-specific builds
    -b, --build             Force build from source (skip binary download)

EXAMPLES:
    # Install latest version from prebuilt binary
    ./install.sh

    # Install specific version
    ./install.sh --version v0.1.0-rc.5

    # Install with ROS2 Jazzy support
    ./install.sh --ros-distro jazzy

    # Force build from source with features
    ./install.sh --build --features embed-ros2-native,embed-ros1-native

    # Install to custom directory
    ./install.sh --dir /usr/local/bin

For more information, visit: https://uos.github.io/minot/installation.html

EOF
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
        VERSION=$(curl -sSf "${GITHUB_API}/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
    elif command -v wget >/dev/null 2>&1; then
        VERSION=$(wget -qO- "${GITHUB_API}/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
    else
        error "Neither curl nor wget found. Please install one of them."
    fi
    
    if [ -z "$VERSION" ]; then
        error "Failed to fetch latest version from GitHub"
    fi
    
    info "Latest version: $VERSION"
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
    
    cd "$TMP_DIR"
    
    if command -v curl >/dev/null 2>&1; then
        curl -sSfL "$URL" -o "$ARCHIVE_FILE" || error "Failed to download binary"
    elif command -v wget >/dev/null 2>&1; then
        wget -q "$URL" -O "$ARCHIVE_FILE" || error "Failed to download binary"
    fi
    
    info "Extracting archive..."
    tar xzf "$ARCHIVE_FILE" || error "Failed to extract archive"
    
    # Create install directory if it doesn't exist
    mkdir -p "$INSTALL_DIR"
    
    info "Installing binaries to $INSTALL_DIR..."
    
    # Install all binaries from the archive
    for binary in "$ARCHIVE_NAME"/*; do
        if [ -f "$binary" ] && [ -x "$binary" ]; then
            BINARY_NAME=$(basename "$binary")
            cp "$binary" "$INSTALL_DIR/$BINARY_NAME"
            chmod +x "$INSTALL_DIR/$BINARY_NAME"
            success "Installed: $BINARY_NAME"
        fi
    done
    
    # Install library files if present
    for lib_file in "$ARCHIVE_NAME"/*.a "$ARCHIVE_NAME"/*.so "$ARCHIVE_NAME"/*.dylib "$ARCHIVE_NAME"/*.h; do
        if [ -f "$lib_file" ]; then
            FILENAME=$(basename "$lib_file")
            cp "$lib_file" "$INSTALL_DIR/$FILENAME"
            success "Installed library: $FILENAME"
        fi
    done
    
    cd - >/dev/null
}

# Check if Rust is installed
check_rust() {
    if ! command -v cargo >/dev/null 2>&1; then
        warn "Rust is not installed on your system."
        printf "\nTo build Minot from source, you need to install Rust.\n"
        printf "Visit: ${BLUE}https://rustup.rs/${NC}\n\n"
        printf "Quick install command:\n"
        printf "  ${GREEN}curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh${NC}\n\n"
        
        printf "Would you like to continue without installing? [y/N] "
        read -r response
        case "$response" in
            [yY][eE][sS]|[yY]) 
                ;;
            *)
                exit 1
                ;;
        esac
        return 1
    fi
    return 0
}

# Build from source
build_from_source() {
    info "Building Minot from source..."
    
    if ! check_rust; then
        error "Rust is required to build from source"
    fi
    
    # Check if git is available
    if ! command -v git >/dev/null 2>&1; then
        error "Git is required to build from source. Please install git."
    fi
    
    TMP_DIR=$(mktemp -d)
    trap "rm -rf '$TMP_DIR'" EXIT
    
    cd "$TMP_DIR"
    
    info "Cloning repository..."
    if [ -n "$VERSION" ]; then
        git clone --depth 1 --branch "$VERSION" "https://github.com/${REPO}.git" minot || error "Failed to clone repository"
    else
        git clone --depth 1 "https://github.com/${REPO}.git" minot || error "Failed to clone repository"
    fi
    
    cd minot
    
    # Determine features to build with
    BUILD_FEATURES="$FEATURES"
    if [ -z "$BUILD_FEATURES" ]; then
        # Default to embed-coord if no features specified
        BUILD_FEATURES="embed-coord"
    fi
    
    info "Building with features: $BUILD_FEATURES"
    
    # Build minot
    cargo build --release --package minot --features "$BUILD_FEATURES" || error "Build failed"
    
    # Create install directory if it doesn't exist
    mkdir -p "$INSTALL_DIR"
    
    info "Installing binaries to $INSTALL_DIR..."
    
    # Install minot binary
    if [ -f "target/release/minot" ]; then
        cp target/release/minot "$INSTALL_DIR/minot"
        chmod +x "$INSTALL_DIR/minot"
        success "Installed: minot"
    fi
    
    # Install minot-coord if it exists
    if [ -f "target/release/minot-coord" ]; then
        cp target/release/minot-coord "$INSTALL_DIR/minot-coord"
        chmod +x "$INSTALL_DIR/minot-coord"
        success "Installed: minot-coord"
    fi
    
    cd - >/dev/null
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

# Main installation logic
main() {
    parse_args "$@"
    
    info "Minot Installation Script"
    printf "\n"
    
    # Detect system
    OS_NAME=$(detect_os)
    ARCH_NAME=$(detect_arch)
    
    # Prefer GNU targets for ROS builds
    PREFER_GNU=0
    if [ -n "$ROS_DISTRO" ]; then
        PREFER_GNU=1
    fi
    
    TARGET=$(get_target_triple "$OS_NAME" "$ARCH_NAME" "$PREFER_GNU")
    
    info "Detected system: $OS_NAME ($ARCH_NAME)"
    info "Target triple: $TARGET"
    
    # Get version if not specified
    if [ -z "$VERSION" ]; then
        get_latest_version
    fi
    
    # Try to download prebuilt binary unless forced to build
    if [ $FORCE_BUILD -eq 0 ]; then
        # Determine archive name based on ROS distro
        if [ -n "$ROS_DISTRO" ]; then
            ARCHIVE_NAME="minot-${ROS_DISTRO}-${TARGET}"
            info "Looking for ROS ${ROS_DISTRO} prebuilt binary..."
        else
            ARCHIVE_NAME="minot-${TARGET}"
            info "Looking for prebuilt binary..."
        fi
        
        if check_binary_exists "$TARGET" "$ARCHIVE_NAME" "$VERSION"; then
            success "Found prebuilt binary!"
            download_and_install "$TARGET" "$ARCHIVE_NAME" "$VERSION"
        else
            warn "No prebuilt binary found for your system"
            
            # If ROS distro was specified but no binary found, inform user
            if [ -n "$ROS_DISTRO" ]; then
                info "ROS ${ROS_DISTRO} build not available for ${TARGET}"
            fi
            
            printf "\nWould you like to build from source? [Y/n] "
            read -r response
            case "$response" in
                [nN][oO]|[nN])
                    error "Installation cancelled"
                    ;;
                *)
                    FORCE_BUILD=1
                    ;;
            esac
        fi
    fi
    
    # Build from source if necessary
    if [ $FORCE_BUILD -eq 1 ]; then
        build_from_source
    fi
    
    printf "\n"
    success "Installation complete!"
    
    # Check if binaries are accessible
    suggest_path_update
    
    printf "\nVerify installation by running:\n"
    printf "  ${GREEN}minot --help${NC}\n\n"
    
    printf "For more information, visit:\n"
    printf "  ${BLUE}https://uos.github.io/minot/${NC}\n\n"
}

# Run main function
main "$@"
