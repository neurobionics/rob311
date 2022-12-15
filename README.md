<img src="https://github.com/neurobionics/rob311/blob/main/rob311.png" width="1024">

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

## Fall 2022

<img src="https://github.com/neurobionics/rob311/blob/main/fall_2022.png" width="1024">
