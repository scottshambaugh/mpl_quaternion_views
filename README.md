This repo holds the code to read in quaternions from an ICM-20948 IMU that's connected to a Teensy 4.0 microcontroller. That is piped to my laptop running python on linux on windows via Windows Subsystem for Linux. If you have a pure windows/mac/linux machine then the steps below will be different but hopefully similar.

## Installation instructions

### ICM-20948 and Teeny Driver:

Get the ICM-20948 driver for the teensy from here: https://github.com/ZaneL/Teensy-ICM-20948

Add this [as a zip library](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries) in the Arduino IDE.

Flash QuaternionAnimation.iso to the teensy. If no connection, make sure microusb cable is data + power and not power only.

### USB Connection for WSL:

Attaching a USB connection from windows to linux in WSL is a bit of a pain. Here's a summary of these directions
https://docs.microsoft.com/en-us/windows/wsl/connect-usb

1) In bash, install needed packages:
```
sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
```

2) In powershell administrator mode, find and attach the teensy:
```
usbipd wsl list
# usbipd wsl attach --busid <busid>
usbipd wsl attach --busid 1-7
```

3) In bash, check that you can seen the device:
```
lsusb  # 'Bus 001 Device 002: ID 16c0:0483 Van Ooijen Technische Informatica Teensyduino Serial'
# This maps to /dev/bus/usb/001/002
```

4) In bash, give read permissions to the USB device:

```
sudo chmod 666 /dev/ttyACM0  
```

5) This isn't working for some reason, but udev rules should keep you from having to doing step 4 every time you reattach the USB devide to WSL.
Make file `/etc/udev/rules.d/0_teensy_udev.rules` and add [copy in this file](https://github.com/arduino/OpenOCD/blob/master/contrib/60-openocd.rules).
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTR{idProduct}=="0483", MODE="0660", GROUP="plugdev"
```

Then in bash:
```
sudo service udev restart
sudo udevadm control --reload
```

## Running

Run `quaternion_plotter.py`.
