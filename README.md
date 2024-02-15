# Enhanced Audio Codec ZL38063 for Raspberry Pi (SaraKIT)

This repository hosts the enhanced audio codec ZL38063 for the Raspberry Pi, specifically tailored for use with the SaraKIT. Building upon the base provided by `/source/sound/soc/codecs/zl38060.c`, this project aims to extend the capabilities and performance of the Raspberry Pi's audio processing, particularly for applications requiring precise and high-quality audio input and output.

## Features

- Improved Automatic Speech Recognition (ASR) performance in far-field applications.
- Enhanced noise reduction algorithms for clearer audio in noisy environments.
- Customizable acoustic echo cancellation for better duplex communication.
- Extended support for voice commands and voice-controlled applications.
- Optimized for use with SaraKIT's ZL38063 audio processor.

Certainly! Below is the revised README section incorporating your instructions and the note about not needing to install the codec if using the provided files.

---

## Installation

**Note:** Installing the codec as described here is not necessary for most users. We strive to provide a ready-to-use codec file for each known version of the Raspberry Pi 64-bit OS. However, if you wish to make changes or add your own controls, the compilation instructions for your kernel version are provided below.

### Prerequisites

First, ensure your system is up to date and has the necessary tools for building kernel modules:

```sh
sudo apt-get update
sudo apt-get install git gcc make bc bison flex libssl-dev libncurses5-dev
```

### Cloning the Kernel Source

```sh
git clone https://github.com/RPi-Distro/rpi-source
cd rpi-source
python rpi-source
cd ..
cd linux
```

### Integrating the Custom Codec

Copy our `zl38060.c` file to the appropriate directory:

`/sound/soc/codecs/zl38060.c`

### Configuring the Kernel

Prepare the default configuration for BCM2711 (used by Raspberry Pi 4 and similar models):

```sh
sudo make bcm2711_defconfig
```

To include the ZL38060 codec in the build, which is not selected by default in the Linux kernel configuration, run the following:

```sh
sudo make menuconfig
```

Navigate to `Device Drivers > Sound card support > Advanced Linux Sound Architecture > ALSA for SoC audio support > CODEC drivers` and select the ZL38060 codec.

### Building the Module

Prepare the module build system and compile the codec module:

```sh
sudo make modules_prepare
sudo make M=sound/soc/codecs
```

### Installing the Module

Copy the `snd-soc-zl38060.ko` file to the appropriate directory and update module dependencies:

```sh
sudo depmod -a
```

Finally, reboot your system:

```sh
sudo reboot
```

**Recommendation:** For use with SaraKIT, we recommend running the latest 64-bit version of the Raspberry Pi OS (Bullseye) - currently version 5.15.84, or (Bookworm) but at a minimum of version 6.1.74-v8+. We have been unable to compile the driver on versions lower (Bookworm) than this at the current time.

---

This revised section provides clear instructions for users who might want to compile and install the custom audio codec themselves while highlighting that for most users, this step is unnecessary thanks to the pre-compiled codec files provided for known Raspberry Pi 64-bit versions.

## Usage

After installation, the enhanced codec can be utilized by any application requiring audio input/output on the Raspberry Pi. For specific usage with SaraKIT, refer to the provided examples and documentation.

## Examples

This repository includes examples demonstrating how to utilize the enhanced codec with SaraKIT for various applications. Check the `examples` directory:
- https://sarakit.saraai.com/getting-started/audio

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

## Acknowledgments

This project builds upon the work done by the Linux kernel community and Microsemi. Special thanks to everyone involved in developing and maintaining the original ZL38060 codec.

## Contact

For questions or support regarding the enhanced audio codec, please open an issue in this repository.

---

Remember to replace placeholder texts like `[repository URL]`, `[repository directory]`, and specific compilation instructions with actual data relevant to your project.
