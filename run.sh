#! /bin/bash

#create image
# docker build -t robot_image .

#run image
docker run --net=host --device /dev/ttyUSB0 --device /dev/gpiochip0 -v /sys/class/pwm/pwmchip2:/sys/class/pwm/pwmchip2 -it --rm --name robot robot_image
