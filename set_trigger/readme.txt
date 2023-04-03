Trigger Mode Control Application
****************************

Basic Configuration:
********************
Dependencies:
-------------

Following packages need to be installed:
 * libudev-dev
 * libusb-1.0-0-dev
 
Install them using:
 $ sudo apt-get install libudev-dev


Build Configuration with src:
-----------------------------
  * Open src folder in command line and build the application using the following 
    command:
    $ gcc Trigger_Mode.c -ludev -o Trigger_Mode
  
Executing:
----------
  * Run by using executable file 
	$ sudo ./Trigger_Mode

NOTE:
-----
To change the Trigger_Mode while streaming the cameras using any streaming application,


