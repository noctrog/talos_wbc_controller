NAME=ros_melodic_talos_wbc

# Create the docker image
docker built -t $NAME

# Start a terminal with mounted volumes
docker run -it --net=host \
    --volume="${PWD}/talos_wbc_controller":"/talos_public_workspace/src/talos_wbc_controller":rw \
    --volume="${PWD}/xpp_talos":"/talos_public_workspace/src/xpp_talos":rw \
    --volume="${PWD}/osqp":"/talos_public_workspace/src/osqp":rw \
    $NAME bash # TODO change container name\
