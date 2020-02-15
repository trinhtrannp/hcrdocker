#!/bin/bash
CWD=(`pwd`)

function askreboot {
	for (( ; ; ))
	do
		read -n 1 -p "Do you want to reboot (Y/n):" ANSWER
		if [ "$ANSWER" == "Y" ]
		then
			sudo reboot
			break
		elif [ "$ANSWER" == "n" ]
		then
			echo "You should reboot in order for changes to take effects."
			break
		else
			echo "Only Y or n is accepted!"
		fi
	done
}

function active {
	# disable services
	cat $CWD/.disabled-services | xargs -I{} sudo systemctl disable {}

	# disable desktop environment
	sudo systemctl set-default multi-user.target

	# finally disable CPU
	echo 0 | sudo tee /sys/devices/system/cpu/cpu7/online
	echo 0 | sudo tee /sys/devices/system/cpu/cpu6/online
	echo 0 | sudo tee /sys/devices/system/cpu/cpu5/online
	echo 0 | sudo tee /sys/devices/system/cpu/cpu4/online
	echo 0 | sudo tee /sys/devices/system/cpu/cpu3/online
	echo 0 | sudo tee /sys/devices/system/cpu/cpu2/online
	echo 0 | sudo tee /sys/devices/system/cpu/cpu1/online

	askreboot
}

function deactive {
	# first enable cpu
	echo 1 | sudo tee /sys/devices/system/cpu/cpu1/online
	echo 1 | sudo tee /sys/devices/system/cpu/cpu2/online
	echo 1 | sudo tee /sys/devices/system/cpu/cpu3/online
	echo 1 | sudo tee /sys/devices/system/cpu/cpu4/online
	echo 1 | sudo tee /sys/devices/system/cpu/cpu5/online
	echo 1 | sudo tee /sys/devices/system/cpu/cpu6/online
	echo 1 | sudo tee /sys/devices/system/cpu/cpu7/online

	# enable services
bach 	cat $CWD/.disabled-services | xargs -I{} sudo systemctl enable {}

	# finally enable desktop environment
	sudo systemctl set-default graphical.target

	askreboot
}

case "$1" in
	"start")
		active
		;;
	"stop")
		deactive
		;;
	*)
		echo "start|stop"
		;;
esac
