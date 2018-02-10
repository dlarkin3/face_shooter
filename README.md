# Use extreme caution when using a autonomously shooting loaded gun.
## This code controlls a pan tilt machanism with an airsoft rifle mounted on it. It locates a human face and then shoots it.

## Create an alias for each device
1. `sudo su`Setting up UDEV rules to alias Arduino and the Dynamixel controllers
2. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="0043", ATTRS{idVendor}=="2341", SYMLINK+="arduino", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-arduino.rules`
3. `echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="06a7", ATTRS{idVendor}=="16d0", SYMLINK+="dynamixel", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-dynamixel.rules`
4. `udevadm control --reload-rules`

## To manually adjust the pan or tilt from cli:
* `rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- 0.5`
* `rostopic pub -1 /pan_controller/command std_msgs/Float64 -- 0.5`

