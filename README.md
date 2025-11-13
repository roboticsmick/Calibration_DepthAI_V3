# Calibration_DepthAI_V3

Calibration script for the OAK-FCC-3P board using DepthAI V3. 

1. Install DepthAI-core V3

    ```bash
    vim install_depthai_v3.sh
    ```
    
    ```bash
    #!/bin/bash
    
    ################################################################################
    # DepthAI Fresh Installation Script for Jetson Orin NX (Ubuntu 22.04)
    #
    # Purpose:
    #   Performs a clean installation of DepthAI library with both C++ and Python
    #   bindings. Removes old installations, installs dependencies, and builds
    #   the library from source with optimized settings for Jetson platform.
    #
    #   Build strategy: Uses vcpkg for internal dependencies (lz4, protobuf, etc.)
    #   but uses system OpenCV (from ROS2/Jetson SDK) for better compatibility.
    #
    # Usage:
    #   chmod +x install_depthai_jetson.sh
    #   ./install_depthai_v3.sh
    #
    # Optional Usage with custom install path:
    #   ./install_depthai_v3.sh /path/to/install/dir
    #
    # Requirements:
    #   - Ubuntu 22.04 (Jetson Orin NX)
    #   - Internet connection
    #   - Sudo privileges
    #   - At least 4GB free disk space
    #
    # Build configuration:
    #   - Uses vcpkg for internal dependencies (lz4, protobuf, etc.)
    #   - Uses system OpenCV (from ROS2/Jetson SDK)
    #   - Estimated build time: 25-40 minutes
    #
    # Directory structure:
    #   ~/depthai-core/              - Source code and build
    #   ~/depthai-core/venv/         - Python virtual environment
    #   ~/depthai_install/           - C++ library installation
    #
    # Author: Generated for Jetson Orin NX Platform
    # Date: 2025-11-08
    ################################################################################
    
    set -e  # Exit on any error
    set -u  # Exit on undefined variable
    set -o pipefail  # Exit on pipe failures
    
    ################################################################################
    # Configuration Variables
    ################################################################################
    
    # Default installation directory
    INSTALL_DIR="${1:-${HOME}/depthai_install}"
    
    # Repository and build settings
    DEPTHAI_REPO="https://github.com/luxonis/depthai-core.git"
    DEPTHAI_BRANCH="main"
    BUILD_DIR="${HOME}"
    WORKSPACE_DIR="${BUILD_DIR}/depthai-core"
    
    # Build configuration
    NUM_CORES=$(nproc)
    BUILD_TYPE="Release"
    
    # Calculate safe number of parallel jobs based on available memory
    # Rule of thumb: Need ~2GB RAM per parallel C++ compilation job
    TOTAL_MEM_GB=$(free -g | awk '/^Mem:/{print $2}')
    AVAILABLE_MEM_GB=$(free -g | awk '/^Mem:/{print $7}')
    
    # Conservative: use 1 job per 2GB of RAM, but at least 1 and max NUM_CORES
    SAFE_JOBS=$((TOTAL_MEM_GB / 2))
    if [ $SAFE_JOBS -lt 1 ]; then
        SAFE_JOBS=1
    fi
    if [ $SAFE_JOBS -gt $NUM_CORES ]; then
        SAFE_JOBS=$NUM_CORES
    fi
    
    # Use fewer jobs if available memory is low
    if [ $AVAILABLE_MEM_GB -lt 4 ]; then
        SAFE_JOBS=$((SAFE_JOBS / 2))
        if [ $SAFE_JOBS -lt 1 ]; then
            SAFE_JOBS=1
        fi
    fi
    
    # Color codes for output
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    NC='\033[0m' # No Color
    
    ################################################################################
    # Function Definitions
    ################################################################################
    
    # Print colored status messages
    print_status() {
        echo -e "${BLUE}[INFO]${NC} $1"
    }
    
    print_success() {
        echo -e "${GREEN}[SUCCESS]${NC} $1"
    }
    
    print_warning() {
        echo -e "${YELLOW}[WARNING]${NC} $1"
    }
    
    print_error() {
        echo -e "${RED}[ERROR]${NC} $1"
    }
    
    # Error handler
    error_exit() {
        print_error "$1"
        exit 1
    }
    
    # Check if command exists
    command_exists() {
        command -v "$1" >/dev/null 2>&1
    }
    
    # Cleanup function for error handling
    cleanup_on_error() {
        print_error "Installation failed. Cleaning up..."
        cd "${HOME}"
        print_status "Build directory preserved at: ${WORKSPACE_DIR}"
        exit 1
    }
    
    # Set trap for error handling
    trap cleanup_on_error ERR
    
    ################################################################################
    # Pre-flight Checks
    ################################################################################
    
    print_status "Starting DepthAI installation for Jetson Orin NX"
    print_status "Installation directory: ${INSTALL_DIR}"
    print_status "Workspace directory: ${WORKSPACE_DIR}"
    
    # Check for sudo privileges
    if ! sudo -n true 2>/dev/null; then
        print_warning "This script requires sudo privileges for dependency installation"
        sudo -v || error_exit "Failed to obtain sudo privileges"
    fi
    
    # Verify Ubuntu version
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [[ "$VERSION_ID" != "22.04" ]]; then
            print_warning "This script is designed for Ubuntu 22.04. Current version: ${VERSION_ID}"
            read -p "Continue anyway? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
    fi
    
    # Check available memory early
    TOTAL_MEM_GB=$(free -g | awk '/^Mem:/{print $2}')
    AVAILABLE_MEM_GB=$(free -g | awk '/^Mem:/{print $7}')
    SWAP_GB=$(free -g | awk '/^Swap:/{print $2}')
    
    print_status "Memory: ${AVAILABLE_MEM_GB}GB available / ${TOTAL_MEM_GB}GB total, ${SWAP_GB}GB swap"
    
    if [ $AVAILABLE_MEM_GB -lt 3 ] && [ $SWAP_GB -lt 2 ]; then
        print_warning "Low memory: ${AVAILABLE_MEM_GB}GB available, ${SWAP_GB}GB swap"
        print_warning "Build may fail with out-of-memory errors"
        echo ""
        echo "Recommendation: Create swap space before continuing:"
        echo "  sudo fallocate -l 4G /swapfile"
        echo "  sudo chmod 600 /swapfile"
        echo "  sudo mkswap /swapfile"
        echo "  sudo swapon /swapfile"
        echo "  # Make permanent: echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab"
        echo ""
        read -p "Continue without adding swap? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 0
        fi
    fi
    
    ################################################################################
    # Cleanup Old Installations
    ################################################################################
    
    print_status "Checking for old depthai-core installations..."
    
    # Remove old workspace if it exists
    if [ -d "${WORKSPACE_DIR}" ]; then
        print_warning "Found existing depthai-core at: ${WORKSPACE_DIR}"
        read -p "Remove and rebuild? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "${WORKSPACE_DIR}"
            print_success "Removed: ${WORKSPACE_DIR}"
        else
            print_error "Cannot proceed with existing directory"
            exit 1
        fi
    fi
    
    # Find and remove old depthai installations in other locations
    OLD_LOCATIONS=(
        "${HOME}/depthai"
        "${HOME}/depthai_build"
        "/usr/local/depthai-core"
        "/opt/depthai-core"
    )
    
    for location in "${OLD_LOCATIONS[@]}"; do
        if [ -d "$location" ]; then
            print_status "Found old installation at: $location"
            read -p "Remove this directory? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                rm -rf "$location"
                print_success "Removed: $location"
            fi
        fi
    done
    
    ################################################################################
    # Install System Dependencies
    ################################################################################
    
    print_status "Checking system dependencies..."
    
    sudo apt-get update || error_exit "Failed to update package list"
    
    # Check if OpenCV is already installed (from ROS2, Jetson SDK, or system)
    OPENCV_INSTALLED=false
    if pkg-config --exists opencv4 2>/dev/null || pkg-config --exists opencv 2>/dev/null; then
        OPENCV_VERSION=$(pkg-config --modversion opencv4 2>/dev/null || pkg-config --modversion opencv 2>/dev/null || echo "unknown")
        print_success "OpenCV already installed (version: ${OPENCV_VERSION})"
        print_status "Using existing OpenCV installation - will not modify"
        OPENCV_INSTALLED=true
    elif [ -f "/usr/include/opencv4/opencv2/opencv.hpp" ] || [ -f "/usr/include/opencv2/opencv.hpp" ]; then
        print_success "OpenCV headers found in system paths"
        print_status "Using existing OpenCV installation"
        OPENCV_INSTALLED=true
    else
        print_warning "OpenCV not detected - DepthAI build may require it"
        print_status "If build fails, install OpenCV manually for Jetson"
    fi
    
    # Core build dependencies
    # vcpkg will handle internal deps (lz4, protobuf, nlohmann-json, etc.)
    # These are the essentials needed for the build system and USB support
    DEPENDENCIES=(
        build-essential
        cmake
        git
        pkg-config
        libudev-dev
        python3
        python3-pip
        python3-venv
        python3-dev
        libusb-1.0-0-dev
        wget
        curl
        libcurl4-openssl-dev
        libssl-dev
    )
    
    # Install dependencies (gracefully handle failures for non-critical packages)
    print_status "Installing/verifying required dependencies..."
    FAILED_PACKAGES=()
    
    for dep in "${DEPENDENCIES[@]}"; do
        if dpkg -l 2>/dev/null | grep -q "^ii  $dep"; then
            print_status "$dep already installed ✓"
        else
            print_status "Installing $dep..."
            if sudo apt-get install -y "$dep" 2>/dev/null; then
                print_success "Installed $dep ✓"
            else
                print_warning "Failed to install $dep (will attempt build anyway)"
                FAILED_PACKAGES+=("$dep")
            fi
        fi
    done
    
    # Report any failed packages
    if [ ${#FAILED_PACKAGES[@]} -gt 0 ]; then
        print_warning "The following packages could not be installed:"
        for pkg in "${FAILED_PACKAGES[@]}"; do
            echo "  - $pkg"
        done
        print_status "Build will continue - vcpkg may handle missing dependencies"
    fi
    
    # Verify CMake version
    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
    print_status "CMake version: ${CMAKE_VERSION}"
    
    # Check if CMake version is >= 3.20
    REQUIRED_CMAKE="3.20"
    if ! command_exists cmake || [ "$(printf '%s\n' "$REQUIRED_CMAKE" "$CMAKE_VERSION" | sort -V | head -n1)" != "$REQUIRED_CMAKE" ]; then
        print_warning "CMake version >= 3.20 required. Current: ${CMAKE_VERSION}"
        print_status "Installing newer CMake..."
        
        wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
            gpg --dearmor - | \
            sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
        
        echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | \
            sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
        
        sudo apt-get update
        sudo apt-get install -y cmake || error_exit "Failed to install CMake"
    fi
    
    print_success "All system dependencies checked"
    
    ################################################################################
    # Clone DepthAI Repository
    ################################################################################
    
    print_status "Cloning depthai-core repository to ${WORKSPACE_DIR}..."
    cd "${BUILD_DIR}"
    
    git clone --recursive --depth 1 --branch "${DEPTHAI_BRANCH}" "${DEPTHAI_REPO}" || \
        error_exit "Failed to clone repository"
    
    cd "${WORKSPACE_DIR}"
    
    print_status "Updating submodules..."
    git submodule update --init --recursive || error_exit "Failed to update submodules"
    
    print_success "Repository cloned successfully"
    
    ################################################################################
    # Build C++ Library
    ################################################################################
    
    print_status "Configuring C++ build..."
    print_status "Using vcpkg for internal dependencies (lz4, protobuf, etc.)"
    print_status "Using system OpenCV and other interface libraries"
    
    # Create build directory
    mkdir -p build
    cd build
    
    # Configure CMake with Jetson-optimized settings
    # Using vcpkg ONLY for internal dependencies (like lz4, protobuf)
    # but using system OpenCV and other interface libraries
    cmake -S .. -B . \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
        -DBUILD_SHARED_LIBS=ON \
        -DDEPTHAI_BUILD_EXAMPLES=ON \
        -DDEPTHAI_OPENCV_SUPPORT=ON \
        -DDEPTHAI_ENABLE_LIBUSB=ON \
        -DDEPTHAI_PCL_SUPPORT=OFF \
        -DDEPTHAI_BUILD_TESTS=OFF \
        -DDEPTHAI_BUILD_DOCS=OFF \
        -DDEPTHAI_BOOTSTRAP_VCPKG=ON \
        -DDEPTHAI_VCPKG_INTERNAL_ONLY=ON \
        || error_exit "CMake configuration failed"
    
    print_status "Building C++ library..."
    print_status "Total CPU cores: ${NUM_CORES}, Using ${SAFE_JOBS} parallel jobs (memory-safe)"
    print_status "Available memory: ${AVAILABLE_MEM_GB}GB, Total: ${TOTAL_MEM_GB}GB"
    if [ $SAFE_JOBS -lt $NUM_CORES ]; then
        print_warning "Reduced parallel jobs to prevent out-of-memory errors"
    fi
    print_status "This will take approximately 30-50 minutes with ${SAFE_JOBS} jobs"
    
    cmake --build . --parallel "${SAFE_JOBS}" || {
        print_error "Build failed. If this was due to memory, try:"
        echo "  cd ${WORKSPACE_DIR}/build"
        echo "  cmake --build . --parallel 1"
        exit 1
    }
    
    print_status "Installing C++ library to ${INSTALL_DIR}..."
    cmake --build . --target install --parallel "${SAFE_JOBS}" || error_exit "C++ installation failed"
    
    print_success "C++ library built and installed successfully"
    
    ################################################################################
    # Verify C++ Installation
    ################################################################################
    
    print_status "Verifying C++ installation..."
    cd "${WORKSPACE_DIR}"
    
    if [ -d "tests/integration" ]; then
        mkdir -p build_integration
        cd build_integration
        
        cmake -S ../tests/integration -B . \
            -DCMAKE_PREFIX_PATH="${INSTALL_DIR}" \
            || print_warning "Integration test configuration failed (non-critical)"
        
        cd "${WORKSPACE_DIR}"
    fi
    
    ################################################################################
    # Build Python Bindings
    ################################################################################
    
    print_status "Setting up Python environment..."
    
    # Create virtual environment for Python bindings
    VENV_DIR="${WORKSPACE_DIR}/venv"
    python3 -m venv "${VENV_DIR}" || error_exit "Failed to create virtual environment"
    
    # Activate virtual environment
    source "${VENV_DIR}/bin/activate" || error_exit "Failed to activate virtual environment"
    
    print_status "Upgrading pip..."
    pip install --upgrade pip setuptools wheel || error_exit "Failed to upgrade pip"
    
    ################################################################################
    # Install Python Dependencies and DepthAI Package
    ################################################################################
    
    print_status "Installing Python dependencies and DepthAI package..."
    cd "${WORKSPACE_DIR}"
    
    if [ -f "examples/python/install_requirements.py" ]; then
        python3 examples/python/install_requirements.py || error_exit "Failed to install Python dependencies"
        print_success "Python dependencies and DepthAI package installed"
    else
        error_exit "install_requirements.py not found in examples/python/"
    fi
    
    # Test Python import
    print_status "Testing Python installation..."
    DEPTHAI_VERSION=$(python3 -c "import depthai as dai; print(dai.__version__)" 2>/dev/null || echo "unknown")
    if [ "$DEPTHAI_VERSION" != "unknown" ]; then
        print_success "DepthAI Python package working - version: ${DEPTHAI_VERSION}"
    else
        print_error "Python import test failed"
    fi
    
    ################################################################################
    # Configure OpenBLAS for ARM (Jetson-specific)
    ################################################################################
    
    print_status "Configuring OpenBLAS for ARM architecture..."
    
    # Check if already in .bashrc
    if ! grep -q "OPENBLAS_CORETYPE" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# OpenBLAS configuration for ARM (prevents illegal instruction errors)" >> ~/.bashrc
        echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
        print_success "OpenBLAS configuration added to ~/.bashrc"
    else
        print_status "OpenBLAS already configured in ~/.bashrc"
    fi
    
    # Set for current session
    export OPENBLAS_CORETYPE=ARMV8
    
    ################################################################################
    # Post-Installation Configuration
    ################################################################################
    
    print_status "Configuring udev rules for USB access..."
    
    # Create udev rules for USB device access
    UDEV_RULES_FILE="/etc/udev/rules.d/80-movidius.rules"
    if [ ! -f "${UDEV_RULES_FILE}" ]; then
        sudo tee "${UDEV_RULES_FILE}" > /dev/null << 'EOF'
    # DepthAI / OAK USB Rules
    SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
    EOF
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        print_success "USB udev rules installed"
    else
        print_status "USB udev rules already exist"
    fi
    
    # Add user to plugdev group if not already
    if ! groups | grep -q plugdev; then
        print_status "Adding user to plugdev group..."
        sudo usermod -a -G plugdev $USER
        print_warning "Added to plugdev group - logout/login required for USB access"
    else
        print_status "User already in plugdev group"
    fi
    
    ################################################################################
    # Create Environment Setup Script
    ################################################################################
    
    print_status "Creating environment setup script..."
    
    ENV_SCRIPT="${INSTALL_DIR}/setup_env.sh"
    cat > "${ENV_SCRIPT}" << EOF
    #!/bin/bash
    # DepthAI Environment Setup Script
    # Source this file to configure your environment for DepthAI
    
    export DEPTHAI_INSTALL_DIR="${INSTALL_DIR}"
    export PATH="\${DEPTHAI_INSTALL_DIR}/bin:\${PATH}"
    export LD_LIBRARY_PATH="\${DEPTHAI_INSTALL_DIR}/lib:\${LD_LIBRARY_PATH}"
    export CMAKE_PREFIX_PATH="\${DEPTHAI_INSTALL_DIR}:\${CMAKE_PREFIX_PATH}"
    
    # Python virtual environment
    export DEPTHAI_VENV="${VENV_DIR}"
    
    # OpenBLAS for ARM
    export OPENBLAS_CORETYPE=ARMV8
    
    # Activate Python virtual environment
    alias activate_depthai="source \${DEPTHAI_VENV}/bin/activate"
    
    echo "DepthAI environment configured"
    echo "Python virtual environment: \${DEPTHAI_VENV}"
    echo "C++ install directory: \${DEPTHAI_INSTALL_DIR}"
    echo ""
    echo "To activate Python environment, run: activate_depthai"
    EOF
    
    chmod +x "${ENV_SCRIPT}"
    
    ################################################################################
    # Installation Summary
    ################################################################################
    
    # Deactivate venv for summary
    deactivate 2>/dev/null || true
    
    print_success "================================"
    print_success "DepthAI Installation Complete!"
    print_success "================================"
    echo ""
    print_status "Installation Summary:"
    echo "  - Workspace: ${WORKSPACE_DIR}"
    echo "  - C++ Library: ${INSTALL_DIR}"
    echo "  - Python Environment: ${VENV_DIR}"
    echo "  - Environment Script: ${ENV_SCRIPT}"
    echo ""
    print_status "Build Configuration:"
    echo "  - vcpkg: Internal dependencies only (lz4, protobuf, etc.)"
    echo "  - OpenCV: System library (from ROS2/Jetson)"
    echo "  - USB Support: Enabled"
    echo "  - OpenBLAS: Configured for ARM"
    echo ""
    print_status "IMPORTANT - USB Permissions:"
    if ! groups | grep -q plugdev; then
        print_warning "You were added to the plugdev group"
        print_warning "You MUST logout/login (or reboot) for USB device access"
    fi
    echo "  1. Logout and login (or reboot)"
    echo "  2. Unplug and replug your OAK device"
    echo ""
    print_status "Usage:"
    echo "  1. Activate Python environment:"
    echo "     cd ${WORKSPACE_DIR}"
    echo "     source venv/bin/activate"
    echo ""
    echo "  2. Test Python:"
    echo "     python3 -c 'import depthai as dai; print(dai.__version__)'"
    echo ""
    echo "  3. Run examples:"
    echo "     python3 examples/python/rgb_preview.py"
    echo ""
    echo "  4. C++ examples:"
    echo "     ${WORKSPACE_DIR}/build/examples/cpp/Camera/camera_all"
    echo ""
    print_status "For permanent environment setup, add to ~/.bashrc:"
    echo "  source ${ENV_SCRIPT}"
    echo ""
    if ! groups | grep -q plugdev; then
        print_warning "REMEMBER: Logout/login required for USB access!"
    fi
    print_success "Installation completed successfully!"
    
    exit 0
    ```
    
    ```bash
    chmod +x install_depthai_v3.sh
    ./install_depthai_v3.sh
    ```

2. Clone the calibration script to the python examples folder

    ```bash
    cd ~/depthai-core
    source venv/bin/activate
    cd ~/depthai-core/examples/python
    git clone https://github.com/roboticsmick/Calibration_DepthAI_V3.git
    ```

3. Update the JSON file with the camera position and camera fov parameters.

4. Create a charuco calibration board

    ```bash
    python3 generate_charuco_board.py -nx 12 -ny 9 -s 4.0 -ms 3.0 -o charuco_board.pdf
    ```
   
6. Update the camera json for your camera layout. There are two example camera json files:
    1. 2 x OV9782 mono global shutter cameras with a M12 75° HFOV lens, and 1 x IMX577 with a M12 lens with a 113° FOV and a 2.7mm focal length:

    ```json
    {
        "board_config":
        {
            "name": "OAK-FFC-3P",
            "revision": "R3M0E3",
            "cameras":{
                "CAM_C": {
      "model":"OV9782",
                    "name": "right",
                    "hfov": 75,
                    "type": "mono",
                    "extrinsics": {
                        "to_cam": "CAM_B",
                        "specTranslation": {
                            "x": 9,
                            "y": 0,
                            "z": 0
                        },
                        "rotation":{
                            "r": 0,
                            "p": 0,
                            "y": 0
                        }
                    }
                },
                "CAM_B": {
      "model":"OV9782",
                    "name": "left",
                    "hfov": 75,
                    "type": "mono",
                    "extrinsics": {
                        "to_cam": "CAM_A",
                        "specTranslation": {
                            "x": 0,
                            "y": -7.01,
                            "z": 0
                        },
                        "rotation":{
                            "r": 0,
                            "p": 0,
                            "y": 0
                        }
                    }
                },
                "CAM_A": {
      "model": "IMX577",
                    "name": "middle",
                    "hfov": 113,
                    "type": "color"
                }
    
            },
            "stereo_config":{
                "left_cam": "CAM_B",
                "right_cam": "CAM_C"
            }
        }
    }

    ```

    2. 1 x IMX577 with a C mount lens with a 4mm UC Series Fixed Focal Length Lens with the aperture set to F2.0. We have interpolated a HFOV of 83.6° for the IMX577 sensor size.

    ```json
    {
        "board_config": {
            "name": "OAK-FCC-3P-RGB-ONLY",
            "revision": "R3M0E3",
            "cameras": {
                "CAM_A": {
                    "model": "IMX577",
                    "name": "middle",
                    "hfov": 83.6,
                    "type": "color"
                }
            }
        }
    }
    ```

7. Run the calibration script to calibrate the stereo and RGB cameras.

    ```bash
    python3 calibrate_depthai_v3.py -s 2.44 -brd OAK-FFC-3P-HQ113.json -nx 12 -ny 9 --ssh-preview --debug-vis
    ```

8. Alternatively you can run the script of a previously captured dataset.

   This expects a folder structure like:

   ```bash
    ├── Calibration
    │   ├── calibrate_depthai_v3.py
    │   ├── dataset
    │   │   ├── left
    │   │   ├── right
    │   │   └── middle
    │   ├── depthai_calibration
    │   │   ├── calibration_utils.py
    │   │   ├── dynamic_recalibration.py
    │   │   ├── epipolar_test_online.py
    │   │   ├── __pycache__
    │   │   │   └── calibration_utils.cpython-312.pyc
    │   │   └── reproject_error_fisheye.py
    │   └── OAK-FFC-3P-HQ113.json
   ```

    ```bash
    python3 calibrate_depthai_v3.py -s 2.44 -brd OAK-FFC-3P-HQ113.json -nx 12 -ny 9 --ssh-preview --debug-vis
    ```
