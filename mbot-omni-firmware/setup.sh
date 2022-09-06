echo "[Setup] Apt Installing required packages..."
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
echo "[Setup] Updating submodules..."
cd lib && git submodule update --init
cd pico-sdk && git submodule update --init
cd ../../
echo "[Setup] Done!"
