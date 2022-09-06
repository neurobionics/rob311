# mbot-omni-firmware
Firware to run on the RPi Pico for the MBot Omni.

## Installation

After cloning the repo, cd into the mbot-omni-firmware directory.
Then, run the setup script:

```bash
./setup.sh
```

Which will install dependencies (requires sudo password) and initialize the submodules.

If setup.sh is not executable by default, do the following to enable it:

```bash
sudo chmod +x setup.sh
```

## Building

Build as follows:
```bash
cd build
cmake ..
make
```
