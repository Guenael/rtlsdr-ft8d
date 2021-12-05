# rtlsdr-ft8d -- FT8 daemon for RTL receivers

*WIP -- Need help for testing this project*

![rtlsdr-wsprd](art/rtlsdr-ft8d-web.jpg)

![Workflow Status](https://img.shields.io/github/workflow/status/Guenael/rtlsdr-ft8d/CI)
![Last commit](https://img.shields.io/github/last-commit/Guenael/rtlsdr-ft8d)
![Commit activity](https://img.shields.io/github/commit-activity/m/Guenael/rtlsdr-ft8d)
![Repo stars](https://img.shields.io/github/stars/Guenael/rtlsdr-ft8d?style=social) (no stars, sad...)

## TL;DR

This project aim at decoding FT8 signals using an RTL device, usually connected to a Raspberry Pi.
To install and use your dongle on a Raspberry Pi with a Rasberian OS:

```bash
sudo apt-get -y install build-essential clang cmake libfftw3-dev libusb-1.0-0-dev librtlsdr-dev libcurl4-gnutls-dev ntp
git clone https://github.com/Guenael/rtlsdr-ft8d
cd rtlsdr-ft8d
make
make install
rtlsdr_ft8d -f 2m -c A1XYZ -l AB12cd -g 29
```

## Overview

This non-interactive application allows automatic reporting of FT8 messages on Internet. The inital idea was to allow a small computer like a Raspberry Pi and a RTL-SDR device to send FT8 reports for VHF/UHF bands. This kind of lightweight setup could run continuously without maintenance and help to get additionnal propagation reports. This code is just a glue between RTL libs and an FT8 open source library based on Karlis Goba (YL3JG) work.

This application written in C does:

- A time alignment (15 sec, required NTPd to run on the OS)
- Start the reception using the RTL lib
- Decimate the IQ data (2.4Msps to 4000 sps)
- Decode FT8 signals
- Report any spots on PSKreporter
- Repeat, again and again...

## Installation

  1. Install a Linux compatible distro on your device (ex. Raspbian for RaspberryPi)
  1. Install dependencies & useful tools (for example, NTP for time synchronization). Example with a Debian based like Raspbian:
     ```
     sudo apt-get -y install build-essential clang cmake libfftw3-dev libusb-1.0-0-dev librtlsdr-dev libcurl4-gnutls-dev ntp
     ```
  1. Clone this repository:
     ```
     git clone https://github.com/Guenael/rtlsdr-ft8d
     ```
  1. Build the application:
     ```
     cd rtlsdr-ft8d
     make
     make install
     ```
  1. Start the application with your right paramaters, ex. for the 2m band, with a fake callsign (A1XYZ):
     ```
     rtlsdr_ft8d -f 2m -c A1XYZ -l AB12cd -g 29
     ```

## Tips (for your Raspberry Pi and SDR dongles)

  - Use ferrite bead on the USB cable to limit the QRN
  - Use an external clean power supply
  - Cut off the display (could help to reduce QRN)
    ```
    /opt/vc/bin/tvservice -o
    ```
  - Remove unused modules (for example, /etc/modules: #snd-bcm2835)
  - Use an enclosure, and ground it

## Crystal stability

Most of RTL dongles use a cheap crystal, and frequency drift can effect the decoding & performance. The use of no-name RTL dongle for VHF/UHF bands usually require crystal modification, for a better one. External clock could be also used, like GPSDO or rubidium reference clock, aligned on 28.8MHz.

Some manufacturers intergrate a 0.5ppm TCXO. It's the best second option, after an external clock. Based on my personal experience:

- NooElec NESDR SMART : Works fine out of the box
- RTL-SDR Blog 1PPM TCXO : Works with some drift, require additional mass, or a better enclosure
- Other no-name like : RT820, E4000, FC0012, FC0013, can work, but require modification and drift a lot

Ex: NooElec NESDR SMArt - Premium RTL-SDR w/ Aluminum Enclosure, 0.5PPM TCXO

https://www.nooelec.com/store/nesdr-smart.html
