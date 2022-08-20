
## ICM-20948 and Teeny Driver:

https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

Uncomment line 29 `#define ICM_20948_USE_DMP` in \src\util\ICM_20948_C.h

Add this [as a zip library](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries) in the Arduino IDE.

## USB Connection for WSL:

https://docs.microsoft.com/en-us/windows/wsl/connect-usb

In bash:
```
sudo apt install linux-tools-virtual hwdata
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
```

In powershell administrator mode:
```
usbipd wsl list
#usbipd wsl attach --busid <busid>
usbipd wsl attach --busid 1-7
```

In bash:
```
lsusb  # 'Bus 001 Device 002: ID 16c0:0483 Van Ooijen Technische Informatica Teensyduino Serial'
# This maps to /dev/bus/usb/001/002
```

Make file `/etc/udev/rules.d/0_teensy_udev.rules` and add [copy in this file](https://github.com/arduino/OpenOCD/blob/master/contrib/60-openocd.rules).
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTR{idProduct}=="0483", MODE="0660", GROUP="plugdev"
```

In bash:
```
sudo service udev restart
sudo udevadm control --reload
sudo chmod 666 /dev/ttyACM0  
```