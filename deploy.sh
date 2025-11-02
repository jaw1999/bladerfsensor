#!/bin/bash
# Deployment script for bladeRF Sensor Server
# This script helps deploy the server to the LattePanda

set -e

echo "=================================="
echo "bladeRF Sensor Server Deployment"
echo "=================================="
echo ""

# Prompt for connection details
read -p "Enter LattePanda username [default: user]: " LATTEPANDA_USER
LATTEPANDA_USER=${LATTEPANDA_USER:-user}

read -p "Enter LattePanda IP address [default: 192.168.1.100]: " LATTEPANDA_IP
LATTEPANDA_IP=${LATTEPANDA_IP:-192.168.1.100}

DEPLOY_DIR="/home/$LATTEPANDA_USER/bladerfsensor"
SSH_CONTROL_PATH="/tmp/ssh-bladerf-$$"

echo ""
echo "Target: $LATTEPANDA_USER@$LATTEPANDA_IP:$DEPLOY_DIR"
echo ""

# Check if SSH is available
if ! command -v ssh &> /dev/null; then
    echo "Error: ssh command not found"
    exit 1
fi

# Cleanup function to close SSH connection
cleanup() {
    if [ -S "$SSH_CONTROL_PATH" ]; then
        ssh -o ControlPath="$SSH_CONTROL_PATH" -O exit "$LATTEPANDA_USER@$LATTEPANDA_IP" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Establish master SSH connection (this is where password is entered)
echo "Establishing SSH connection (enter password once)..."
ssh -o ControlMaster=yes -o ControlPath="$SSH_CONTROL_PATH" -o ControlPersist=300 -fN "$LATTEPANDA_USER@$LATTEPANDA_IP"

if [ ! -S "$SSH_CONTROL_PATH" ]; then
    echo "Error: Cannot connect to $LATTEPANDA_USER@$LATTEPANDA_IP"
    echo ""
    echo "Please ensure:"
    echo "  1. LattePanda is powered on and connected to network"
    echo "  2. SSH is enabled on LattePanda"
    echo "  3. You can connect: ssh $LATTEPANDA_USER@$LATTEPANDA_IP"
    echo "  4. Update LATTEPANDA_USER and LATTEPANDA_IP in this script if needed"
    exit 1
fi

echo "Connection successful!"
echo ""

# Create remote directory
echo "Creating remote directory..."
ssh -o ControlPath="$SSH_CONTROL_PATH" "$LATTEPANDA_USER@$LATTEPANDA_IP" "mkdir -p $DEPLOY_DIR/server"

# Copy server files
echo "Copying server files..."
rsync -avz --progress \
    -e "ssh -o ControlPath=$SSH_CONTROL_PATH" \
    --exclude 'build' \
    --exclude '*.o' \
    --exclude '*.a' \
    server/ "$LATTEPANDA_USER@$LATTEPANDA_IP:$DEPLOY_DIR/server/"

echo ""
echo "Files copied successfully!"
echo ""

# Check if dependencies need to be installed
echo "Checking if dependencies need installation..."
NEEDS_INSTALL=$(ssh -o ControlPath="$SSH_CONTROL_PATH" "$LATTEPANDA_USER@$LATTEPANDA_IP" \
    "if ! dpkg -l | grep -q libbladerf-dev || ! dpkg -l | grep -q libliquid-dev; then echo 'yes'; else echo 'no'; fi")

if [ "$NEEDS_INSTALL" = "yes" ]; then
    echo ""
    echo "Dependencies need to be installed. This requires sudo access."
    echo "You will be prompted for your sudo password on the LattePanda."
    echo ""

    # Install dependencies with interactive sudo
    # Note: apt-get update may warn about some repos, but we continue anyway
    ssh -tt -o ControlPath="$SSH_CONTROL_PATH" "$LATTEPANDA_USER@$LATTEPANDA_IP" \
        "sudo apt-get update || true && sudo apt-get install -y cmake build-essential pkg-config libfftw3-dev libbladerf-dev libbladerf2 libliquid-dev"

    echo ""
    echo "Dependencies installed successfully!"
else
    echo "Dependencies already installed."
fi

echo ""
echo "Building on LattePanda..."

# Build without sudo
ssh -o ControlPath="$SSH_CONTROL_PATH" "$LATTEPANDA_USER@$LATTEPANDA_IP" << 'ENDSSH'
set -e

cd ~/bladerfsensor/server

# Build
echo "Building server..."
mkdir -p build
cd build
cmake ..
make -j$(nproc)

echo ""
echo "Build complete!"
ls -lh bladerf_server

ENDSSH

echo ""
echo "=================================="
echo "Deployment Complete!"
echo "=================================="
echo ""
echo "To run the server on LattePanda:"
echo "  ssh $LATTEPANDA_USER@$LATTEPANDA_IP"
echo "  cd $DEPLOY_DIR/server/build"
echo "  ./bladerf_server"
echo ""
echo "Or run remotely:"
echo "  ssh $LATTEPANDA_USER@$LATTEPANDA_IP 'cd $DEPLOY_DIR/server/build && ./bladerf_server'"
echo ""
echo "Web interface (once server is running):"
echo "  Main UI: http://$LATTEPANDA_IP:8080/"
echo ""
