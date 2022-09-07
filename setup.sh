echo "[Setup] Apt Installing required packages..."
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
echo "[Setup] Changing read/write access..."
sudo chown $USER -R .
echo "[Setup] Copying picoload tool..."
sudo cp ./picoload /bin/
echo "[Setup] Creating Build directory..."
mkdir ./mbot-omni-firmware/build
echo "[Setup] Building..."
cd ./mbot-omni-firmware/build && cmake ..
make -j4
echo "[Setup] Creating /mnt/pico..."
mkdir /mnt/pico
