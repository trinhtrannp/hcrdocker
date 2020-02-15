#!/bin/bash
set -e

if [ -z "$MASTERIP" ]
then
    MASTERIP=localhost
fi

host_ip=(`hostname -I`)
export ROS_IP=${host_ip[0]}
export ROS_MASTER_URI=http://$MASTERIP:11311
export HCR_LASER_SENSOR=rplidar

cp -r /hcr/ws_hcr/src/* /ws_hcr/src/
rm -rf /ws_hcr/src/src/

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ws_hcr/devel/setup.bash"

mkdir -p /var/run/couchdb
chown -R couchdb:couchdb /var/run/couchdb/

#find /ws_hcr/src/hcr_navigation/maps/ -type f -name '*.yaml' | xargs -I{} sed -i 's:/home/eik/:/:g' {}
find /ws_hcr/ -type f -name '*.launch' | xargs -I{} sed -i 's:/xacro\.py:/xacro --inorder:g' {}
find /ws_hcr/ -type f -name '*.launch.xml' | xargs -I{} sed -i 's:/xacro\.py:/xacro --inorder:g' {}

function init-ork-db {
    COUCHDB_LOCK=/hcr/couchdb/db/object_recognition.couch

    if [[ -a ${COUCHDB_LOCK} ]]; then
        echo "${COUCHDB_LOCK} exists. The object have already been put into couchdb, there no need to put it there any more" > /hcr/ork-object.log
    else
        OBJ_IDS=(`rosrun object_recognition_core object_add.py -n "IceTea" -d "A universal can of Ice Tea" --commit`)
        rosrun object_recognition_core mesh_add.py ${OBJ_IDS[5]} /ws_hcr/src/ork_tutorials/data/coke.stl --commit > /hcr/ork-object.log
        echo "object /ws_hcr/src/ork_tutorials/data/coke.obj have been put into couchdb with ID: ${OBJ_IDS[5]}" > /hcr/ork-object.log
    fi
}

function start_all {
    nohup $"roscore" -v > /hcr/roscore.log &
    echo "... roscore started"
    sleep 3
    nohup $"couchdb" start > /hcr/couchdb.log &
    echo "... couchdb started"
    sleep 3
    #nohup $"./init-ork-db.sh" > /hcr/ork-object.log &
    init-ork-db
    echo "... ork object configured"
    sleep 3
    nohup $"rosrun" object_recognition_core push.sh > /hcr/couchdb-webui.log &
    echo "... enable webui for couchdb"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_ctrl_base.launch > /hcr/hcr-ctrl-base.log &
    echo "... hcr_ctrl_base started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_navigation.launch > /hcr/hcr-navigation.log &
    echo "... hcr_navigation started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_ctrl_arm.launch > /hcr/ctrl-arm.log &
    echo "... hcr_ctrl_arm started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_ork.launch > /hcr/hcr-ork.log &
    echo "... hcr_ork started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_moveit.launch > /hcr/hcr-moveit.log &
    echo "... hcr_moveit started"
    sleep 3
    nohup $"roslaunch" hcr_state hcr_state.launch > /hcr/hcr-state.log &
    echo "... hcr_state started"
    sleep 3
}

function start_master {
    nohup $"roscore" -v > /hcr/master-roscore.log &
    echo "... roscore started"
    sleep 3
}

function start_slave {
    nohup $"couchdb" start > /hcr/couchdb.log &
    echo "... couchdb started"
    sleep 3
    #nohup $"./init-ork-db.sh" > /hcr/ork-object.log &
    init-ork-db
    echo "... ork object configured"
    sleep 3
    nohup $"rosrun" object_recognition_core push.sh > /hcr/couchdb-webui.log &
    echo "... enable webui for couchdb"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_ctrl_base.launch > /hcr/hcr-ctrl-base.log &
    echo "... hcr_ctrl_base started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_navigation.launch > /hcr/hcr-navigation.log &
    echo "... hcr_navigation started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_ctrl_arm.launch > /hcr/ctrl-arm.log &
    echo "... hcr_ctrl_arm started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_ork.launch > /hcr/hcr-ork.log &
    echo "... hcr_ork started"
    sleep 3
    nohup $"roslaunch" hcr_bringup hcr_moveit.launch > /hcr/hcr-moveit.log &
    echo "... hcr_moveit started"
    sleep 3
    nohup $"roslaunch" hcr_state hcr_state.launch > /hcr/hcr-state.log &
    echo "... hcr_state started"
    sleep 3
}

if [ -z "$MODE" ]
then
    start_all
else
    if [ "$MODE" = "all" ]
    then
        start_all
    elif [ "$MODE" = "master" ]
    then
        start_master
    elif [ "$MODE" = "slave" ]
    then
        start_slave
    fi
fi

trap : TERM INT; sleep infinity & wait
