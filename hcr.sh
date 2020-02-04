##!/bin/bash
CONTAINER_NAME="hcr"

function start {
    if ! docker top $CONTAINER_NAME &>/dev/null
    then
        echo "Container is not running at the moment. Try to cleanup dead container first.".
        docker rm $CONTAINER_NAME &>/dev/null
        echo "Now starting $CONTAINER_NAME ....."
        nohup $"docker-compose" -f ./docker-compose.yml up &
        return 0
    else
        echo "Container $CONTAINER_NAME is still running. Please try to stop it first or do a restart instead."
        return 1
    fi
}

function stop {
    if ! docker top $CONTAINER_NAME &>/dev/null
    then
        echo "Container is already stoped, will remove it."
    else
        echo "Container $CONTAINER_NAME is still running. Now try to stop it."
        docker stop $CONTAINER_NAME &>/dev/null
    fi
    
    docker rm $CONTAINER_NAME &>/dev/null
    return 0
}

case "$1" in
    "start")
        start
        echo "Finished !"
        ;;
    "stop")
        stop
        echo "Finished !"
        ;;
    "restart")
        stop
        sleep 3
        start
        echo "Finished !"
        ;;
    *)
        echo "start|stop|restart"
        ;;
esac