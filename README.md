# ECEn_631_Group_1
Repo for ECEn 631 Winter 2019 Group 1

### Setup 
```shell
git clone <the html/ssh link from the green button in the top right corner titled "Clone or Download">
cd ECEn_631_Group_1/<project>/
mkdir build
cd build
cmake ..
make
./<project executable> 2 # Use "2" to interface to Logitech webcam; leave blank for integrated webcam
```
### Use for Project 1: Visual Inspection

Use keys 1 through 5 for testing different effects.  Press f to turn on object classification.  Press esc to quit.

A video of the session will be automatically generated and put in the output directory titled result.avi.


### Use for Project 2: Plinko

Commands to run for write access to serial port (commanding Arduino):

```
sudo adduser $USER dialout
sudo usermod -a -G dialout $USER
```