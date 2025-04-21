# local testing

#!/bin/bash

# Run OpenDLV vehicle view
echo "Starting openDLV-vehicle-view..."
docker run --rm --init --net=host --name=opendlv-vehicle-view \
  -v "$PWD":/opt/vehicle-view/recordings \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -p 8081:8081 chrberger/opendlv-vehicle-view:v0.0.64 &

sleep 3

# Run h264decoder
echo "Starting h264decoder..."
docker run --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp:/tmp h264decoder:v0.0.5 --cid=253 --name=img &

sleep 3

# Run HSV inspector
echo "Starting hsv-filter microservice..."
docker run --rm -ti --init --ipc=host -v /tmp:/tmp -e DISPLAY=$DISPLAY hsv:latest \
  --name=img --width=640 --height=480 \
  --hue-min=100 --hue-max=140 \
  --sat-min=150 --sat-max=255 \
  --val-min=50 --val-max=255

# Done