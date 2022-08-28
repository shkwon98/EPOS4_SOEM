# ANSUR_SOEM_Controller
This is a real-time EtherCAT master application to control EtherCAT slave devices(e.g. epos4, elmo, ...).

---

## PREREQUISITES
+ This application was developed with Preempt-rt patched Raspberry Pi 4
	+ Developed Preempt-rt linux kernel version is 4.19.y-rt
	+ Developed OS debian version is 10(buster)
	+ For more details, go to the [link](https://github.com/shkwon98/RPI_PreemptRT)
+ To use this application with gui, extra Ethernet connection on Raspberry Pi is required
	+ For example, WIZnet W5500 Ethernet module with SPI interface or wi-fi can be applied
	+ If you are going to use the same solution, you can find out details [here](https://github.com/shkwon98/RPI_PreemptRT)
+ [ANSUR_GUI](https://github.com/shkwon98/ANSUR_SOEM_GUI) installed remote PC is required for remote connection
#### FRAMEWORK:

![framework](https://user-images.githubusercontent.com/86837107/155870476-c4409fc2-9101-4b08-91ce-bc8333312e19.jpg)

---


## Dependency

### Install SOEM
`git clone https://github.com/OpenEtherCATsociety/SOEM`

`cd SOEM`

`mkdir build`

`cd build`

`cmake ..`

`make -j4`

`sudo make install`


## HOW TO LAUNCH
`mkdir build`

`cd build`

`cmake ..`

`make -j4`

`sudo ./ex_robot eth0`
