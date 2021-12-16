# rtlsdr-ft8d -- FT8 daemon for RTL receivers

**WIP -- Need help for testing this project!!**

![rtlsdr-wsprd](art/rtlsdr-ft8d-web.jpg)

![Project Status](https://img.shields.io/badge/status-WIP-red)
![Workflow Status](https://img.shields.io/github/workflow/status/Guenael/rtlsdr-ft8d/CI)
![Last commit](https://img.shields.io/github/last-commit/Guenael/rtlsdr-ft8d)
![Commit activity](https://img.shields.io/github/commit-activity/m/Guenael/rtlsdr-ft8d)
![Repo stars](https://img.shields.io/github/stars/Guenael/rtlsdr-ft8d?style=social)

## TL;DR

This project aim at decoding [FT8](https://en.wikipedia.org/wiki/WSJT_(amateur_radio_software)#FT8) signals using an [RTL device](https://osmocom.org/projects/rtl-sdr/wiki/Rtl-sdr), usually connected to a [Raspberry Pi](https://www.raspberrypi.org/).
To install and use your dongle on a Raspberry Pi with a Raspberry Pi OS, follow these steps:

```bash
echo "== Install dependencies"
sudo apt-get update && sudo apt-get -y install build-essential clang cmake libfftw3-dev libusb-1.0-0-dev libcurl4-gnutls-dev ntp git

echo "== Install rtl-sdr library (on RPi, don't use your distro package)"
git clone https://github.com/osmocom/rtl-sdr
cd rtl-sdr
mkdir -p make
cd make
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DDETACH_KERNEL_DRIVER=ON -Wno-dev ..
make
sudo make install
cd ../..

echo "== Install rtlsdr-ft8d"
git clone https://github.com/Guenael/rtlsdr-ft8d
cd rtlsdr-ft8d
git submodule update --init --recursive
make
sudo make install

echo "== Start/test rtlsdr-ft8d"
rtlsdr_ft8d -f 2m -c A1XYZ -l AB12cd -g 29
```

## Overview

This non-interactive application allows automatic reporting of FT8 messages on Internet with [PSKreporter](https://pskreporter.info/pskmap.html). The inital idea was to allow a small computer like a Raspberry Pi and a RTL-SDR device to send FT8 reports for [VHF/UHF](https://en.wikipedia.org/wiki/Amateur_radio_frequency_allocations#Very_high_frequencies_and_ultra_high_frequencies) bands. This kind of lightweight setup could run continuously without maintenance and help to get additionnal propagation reports. This code is just a glue between RTL libs and an FT8 open source library based on [Karlis Goba (YL3JG) work](https://github.com/kgoba/ft8_lib).

This application written in C does:

- A time alignment (15 sec, required NTPd to run on the OS)
- Start the reception using the RTL lib
- Decimate the IQ data (2.4Msps to 3200 sps)
- Decode FT8 signals
- Report any spots on PSKreporter
- Repeat, again and again...


## Installation

  1. Install a Linux compatible distro on your device.
     
     For Raspberry Pi, you can download official images [here](https://www.raspberrypi.com/software/operating-systems/).
  
  2. It's a good practice to update your OS. With Pi OS, run this command as usual:
     ```bash
     sudo apt-get update && sudo apt-get upgrade
     ```

  3. Install dependencies & useful tools (for example, [NTP](https://en.wikipedia.org/wiki/Network_Time_Protocol) for time synchronization). Example with a Debian based OS, like Rasbian, or Raspberry Pi OS:
     ```bash
     sudo apt-get update && sudo apt-get -y install build-essential clang cmake libfftw3-dev libusb-1.0-0-dev libcurl4-gnutls-dev ntp git
     ```
 
  4. Install `rtl-sdr` library manually. **Do not use the `librtlsdr-dev` package on Raspberry PiOS**. There is a know bug with this lib and rtlsdr_wsprd will not be able to get enough samples (don't decode anything & 100% CPU pattern).
     ```bash
     git clone https://github.com/osmocom/rtl-sdr
     cd rtl-sdr
     mkdir -p make
     cd make
     cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr -DDETACH_KERNEL_DRIVER=ON -Wno-dev ..
     make
     sudo make install
     cd ../..
     ```
  Note: You may have to re-plug you dongle if it was already connected, or play with `udev` if not automatically detected.
  
  5. Clone this repository:
     ```bash
     git clone https://github.com/Guenael/rtlsdr-ft8d
     cd rtlsdr-ft8d
     git submodule update --init --recursive
     ```

  6. Build the application:
     ```bash
     make
     sudo make install
     ```
  
  7. Finally, start the application with the right parameters/options for you (frequency, callsign, locator etc... Fake example below):
     ```bash
     rtlsdr_ft8d -f 2m -c A1XYZ -l AB12cd -g 29
     ```

## Tips (for your Raspberry Pi and SDR dongles)

  - Use ferrite bead on the USB cable to limit the QRN
  - Use an external clean power supply
  - Cut off the display (could help to reduce QRN)
    ```bash
    /opt/vc/bin/tvservice -o
    ```
  - Remove unused modules (for example, /etc/modules: #snd-bcm2835)
  - Use an enclosure, and ground it

## Crystal stability

Most of RTL dongles use a cheap crystal, and frequency drift can effect the decoding & performance. The use of no-name RTL dongle for VHF/UHF bands usually require crystal modification, for a better one. External clock could be also used, like GPSDO or rubidium reference clock, aligned on 28.8MHz.

Some manufacturers integrate a 0.5ppm TCXO. It's the best second option, after an external clock. Based on my personal experience:

- NooElec NESDR SMART : Works fine out of the box
- RTL-SDR Blog 1PPM TCXO : Works with some drift, require additional mass, or a better enclosure
- Other no-name like : RT820, E4000, FC0012, FC0013, can work, but require modification and usually drift a lot
