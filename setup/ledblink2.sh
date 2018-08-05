echo 388 > /sys/class/gpio/export
echo 298 > /sys/class/gpio/export
echo 480 > /sys/class/gpio/export
echo 486 > /sys/class/gpio/export

echo  out > /sys/class/gpio/gpio388/direction
echo  out > /sys/class/gpio/gpio298/direction
echo  out > /sys/class/gpio/gpio480/direction
echo  out > /sys/class/gpio/gpio486/direction

while true
do
  echo 1 > /sys/class/gpio/gpio388/value
sleep 2
  echo 1 > /sys/class/gpio/gpio298/value
sleep 2
  echo 1 > /sys/class/gpio/gpio480/value
sleep .2
  echo 1 > /sys/class/gpio/gpio486/value
  sleep 2

  echo 0 > /sys/class/gpio/gpio388/value
sleep 2
  echo 0 > /sys/class/gpio/gpio298/value
sleep 2
  echo 0 > /sys/class/gpio/gpio480/value
sleep 2
  echo 0 > /sys/class/gpio/gpio486/value
 sleep 2

done

