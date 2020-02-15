##!/bin/bash
CWD=(`pwd`)
CONTAINER_NAME="hcr"

function start_all {
    if ! docker top $CONTAINER_NAME &>/dev/null
    then
        echo "Container is not running at the moment. Try to cleanup dead container first.".
        docker rm $CONTAINER_NAME &>/dev/null
        echo "Now starting $CONTAINER_NAME ....."
        export MODE=all
        nohup $"docker-compose" -f ./docker-compose.yml up &
        return 0
    else
        echo "Container $CONTAINER_NAME is still running. Please try to stop it first or do a restart instead."
        return 1
    fi
}

function start_master {
    if ! docker top $CONTAINER_NAME &>/dev/null
    then
        echo "Container is not running at the moment. Try to cleanup dead container first.".
        docker rm $CONTAINER_NAME &>/dev/null
        echo "Now starting $CONTAINER_NAME ....."
        nohup $"docker" run --detach --name "hcr" --network "hcrnetwork" --network-alias "hcrmaster" --hostname "hcrmaster" --privileged -e MODE=master -e MASTERIP=localhost -v "/dev/":"/dev/" -v "$CWD/":"/hcr/" -v "/bin/ping":"/bin/ping" --entrypoint "/hcr/entrypoint.sh" hcr:kinetic-ros &
        return 0
    else
        echo "Container $CONTAINER_NAME is still running. Please try to stop it first or do a restart instead."
        return 1
    fi
}

function start_slave {
    if ! docker top $CONTAINER_NAME &>/dev/null
    then
        echo "Container is not running at the moment. Try to cleanup dead container first.".
        docker rm $CONTAINER_NAME &>/dev/null
        echo "Now starting $CONTAINER_NAME ....."
        export MODE=slave
        nohup $"docker" run --detach --name "hcr" --network "hcrnetwork" --network-alias "hcrslave" --hostname "hcrslave" --privileged -e MODE=slave -e MASTERIP=hcrmaster -v "/dev/":"/dev/" -v "$CWD/":"/hcr/" -v "/bin/ping":"/bin/ping" --entrypoint "/hcr/entrypoint.sh" hcr:kinetic-ros &
        return 0
    else
        echo "Container $CONTAINER_NAME is still running. Please try to stop it first or do a restart instead."
        return 1
    fi
}

function start_debug {
    if ! docker top $CONTAINER_NAME &>/dev/null
    then
        echo "Container is not running at the moment. Try to cleanup dead container first.".
        docker rm $CONTAINER_NAME &>/dev/null
        echo "Now starting $CONTAINER_NAME ....."
        export MODE=debug
        nohup $"docker" run --detach --name "hcrdebug" --network "hcrnetwork" --network-alias "hcrdebug" --hostname "hcrdebug" --privileged -e MODE=debug -e MASTERIP=localhost -v "/dev/":"/dev/" -v "$CWD/":"/hcr/" -v "/bin/ping":"/bin/ping" --entrypoint "/hcr/entrypoint.sh" hcr:kinetic-ros &
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
    "startall")
        start_all
        echo "Finished !"
        ;;
    "startmaster")
        start_master
        echo "Finished !"
        ;;
    "startslave")
        start_slave
        echo "Finished !"
        ;;
    "startdebug")
        start_debug
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
        echo "startall|startmaster|startslave|startdebug|stop|restart"
        ;;
esac
