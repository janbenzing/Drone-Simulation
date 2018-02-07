ARGS=$@

# Check if container is running
CONTAINER=$(docker container ls -f name=dronecourse -q)

if [ -z "$CONTAINER" ];
then 
    # Error
    echo "[Error] The simulation is not running!";
else
    # Run platform command in container
    docker exec dronecourse platform -h;
    docker exec dronecourse platform $ARGS;
fi;