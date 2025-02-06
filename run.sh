#! /bin/bash

#create image
docker build -t robot_image .

#run image
docker stop robot
docker rm robot
xhost +local:root
docker run --name robot \
        --net=host \
        --device /dev/ttyUSB0 --device /dev/gpiochip0 \
        -v /sys/class/pwm/pwmchip2:/sys/class/pwm/pwmchip2 \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XDG_RUNTIME_DIR=/run/user/$(id -u)" \
        --env="WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/run/user/$(id -u)/$WAYLAND_DISPLAY" \
        --volume="/dev/dri:/dev/dri" \
        -it --rm robot_image