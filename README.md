## Installation

After cloning the repo, run the setup script:

```bash
./setup.sh
```
This setup script will install CMAKE tools and other dependencies that are essential to build the firmware.

## Building the MBot firmware

Build as follows:
```bash
cd ./mbot-omni-firmware/build
cmake ..
make
```

## Flashing the MBot firmware on RaspberryPi Pico

```bash
picoload /dev/sda1
```

Note that the drive name ("sda1") changes everytime you connect the pico to the Rpi.
