#!/bin/bash
function nav {
    #source ./ws_hcr/devel/setup.bash
    nohup $"rviz" -v -d ./ws_hcr/src/hcr_bringup/rviz/nav.rviz &
}

function orknav {
    #source ./ws_hcr/devel/setup.bash
    nohup $"rviz" -v -d ./ws_hcr/src/hcr_bringup/rviz/ork_nav.rviz &
}

function all {
    #source ./ws_hcr/devel/setup.bash
    nohup $"rviz" -v -d ./ws_hcr/src/hcr_bringup/rviz/complete.rviz &
}

function print_help {
    echo "control.sh [ROSCORE_IP] {nav|orknav|all}"
    echo "ROSCORE_IP: the ip address or hostname of the server in which roscore is running."
    echo "nav:      start rviz with navigation functions only"
    echo "orknav:   start rviz with navigation functions and ork functions"
    echo "all: start rviz with all functions"
}

case "$1" in
    "help")
        print_help
        ;;
    *)
        host_ip=(`hostname -I`)
        export ROS_IP=${host_ip[0]}
        export ROS_MASTER_URI=http://$1:11311
        ;;
esac

if [[ ! -z "$2" ]]; then 
    case "$2" in
        "nav")
            nav
            ;;
        "orknav")
            orknav
            ;;
        "all")
            all
            ;;
        *)
            print_help
    esac
fi