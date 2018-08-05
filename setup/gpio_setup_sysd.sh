#!/bin/bash
declare -a pins=("388" "298" "480" "486")

for pin in "${pins[@]}"
    do
        echo "Creating gpio "$pin
        echo $pin > /sys/class/gpio/export
        echo  out > /sys/class/gpio/gpio$pin/direction
        chgrp -HR dialout /sys/class/gpio/gpio$pin/direction
        chmod -R g+rw /sys/class/gpio/gpio$pin/direction
        chgrp -HR dialout /sys/class/gpio/gpio$pin/value
        chmod -R g+rw /sys/class/gpio/gpio$pin/value
    done

