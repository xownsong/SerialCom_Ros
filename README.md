USB2Serial port

Port checking like /dev/ttyUSB0

	~$ dmesg | grep ttyUSB
	>> [    2.393858] usb 1-1: FTDI USB Serial Device converter now attached to ttyUSB0


If it is not checked,

	~$ sudo apt install moserial

Make connection usb2serial

	~$ screen /dev/ttyUSB0 11520

Or 

	~$ sudo apt-get install minicom
	~$ minicom -s


[configuration] --> serial port setup --> Serial Device : /dev/ttyUSB0 --> "Enter"

Save setup as dfi --> /etc/minicom/minicom.dfl


====================================================================================================

~$ sudo apt-get install ros-melodic-serial

====================================================================================================
Ftdi driver install

Ftdi driver has two types of drivers : virtual COM port (VCP) driver and the D2XX API driver. 

This example describes the VCP driver installation. 

	~$ sudo apt-get install putty
	~$ putty

Check "Serial"
Serial line : /dev/ttyUSB0 115200


If you get an error when Putty is trying to open the COM port

It most likely is because your Linux username is not a member of the “dialout” group, to which the port or
tty device (USB to serial adapter) belongs to, so you need to add your username to that group.
You can check if your username is a member of the group or not, with the “groups” command:

	~$ groups dml
	~$ sudo gpsswd --add dml dialout
	~$ groups dml
	>> dml : dml adm dialout cdrom sudo dip plugdev lpadmin sambashare
