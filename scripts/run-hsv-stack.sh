
#!/bin/bash

# Start OpenDLV Vehicle View
gnome-terminal -- bash -c "cd '$PWD'; echo '[OpenDLV] Starting...'; docker run --rm --init --net=host --name=opendlv-vehicle-view \
  -v '$PWD':/opt/vehicle-view/recordings \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -p 8081:8081 chrberger/opendlv-vehicle-view:v0.0.64; exec bash"

sleep 2

# Start h264decoder (producer)
gnome-terminal -- bash -c "cd '$PWD'; echo '[h264decoder] Starting...'; docker run --rm --net=host --ipc=host \
  -e DISPLAY=\$DISPLAY -v /tmp:/tmp h264decoder:v0.0.5 \
  --cid=253 --name=img; exec bash"

# Wait until shared memory file is available
echo "Waiting for /tmp/img shared memory to be created by h264decoder..."
while [ ! -e /tmp/img ]; do
    sleep 1
done
echo "Shared memory found!"

sleep 1

# Start HSV filter (consumer)
gnome-terminal -- bash -c "cd '$PWD'; echo '[HSV filter] Starting...'; docker run --rm --net=host --ipc=host \
  -v /tmp:/tmp -e DISPLAY=\$DISPLAY hsv:latest \
  --name=img --width=640 --height=480 \
  --hue-min=100 --hue-max=140 --sat-min=150 --sat-max=255 --val-min=50 --val-max=255; exec bash"