NAME=ros_talos

# Create the docker image
docker build -t $NAME .

# Start a terminal with mounted volumes
docker run -it --rm -p=6080:80 \
    --volume="${PWD}/talos_wbc_controller":"/home/ubuntu/talos_public_ws/src/talos_wbc_controller":rw \
    --volume="${PWD}/xpp_talos":"/home/ubuntu/talos_public_ws/src/xpp_talos":rw \
    --volume="${PWD}/osqp":"/home/ubuntu/talos_public_ws/src/osqp":rw \
    $NAME bash
