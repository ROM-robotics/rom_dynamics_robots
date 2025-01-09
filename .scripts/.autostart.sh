#!/usr/bin/env bash
SHELL=/bin/bash
PATH=$PATH:/bin:/sbin:/usr/bin:/usr/sbin
sleep 3
# for graphical user interface
#gnome-terminal --full-screen -- bash -c "tmuxinator ; exec bash -i"


# for multi user target
tmuxinator start -p /home/mr_robot/Desktop/Git/rom_dynamics_robots/mytmux/nav2/software_qt/.tmuxinator.yml
exec bash -i
