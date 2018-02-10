Setting up UDEV rules to alias Arduino and the Dynamixel controllers

Create the arduino alias:
echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="0043", ATTRS{idVendor}=="2341", SYMLINK+="arduino", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-arduino.rules


Create dynamixel alias
echo 'SUBSYSTEM=="tty", ATTRS{idProduct}=="06a7", ATTRS{idVendor}=="16d0", SYMLINK+="dynamixel", ACTION=="add", GROUP="dialout", MODE="0660"' >> /etc/udev/rules.d/99-dynamixel.rules

udevadm control --reload-rules


rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- 0.5
rostopic pub -1 /pan_controller/command std_msgs/Float64 -- 0.5

