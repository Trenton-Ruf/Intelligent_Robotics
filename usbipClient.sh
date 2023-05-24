#!/bin/bash

# Start Kernel Module
sudo modprobe vhci-hcd

# Prompt for bus id
echo "Enter the IP Address of the Server"
read ip

# Vendor ID of the Camera
id="046d:0821"

# Get bus ID from the server
busid=$(usbip list -r $ip $id | grep -i $id | cut -d':' -f1)
echo BUS ID is $busid

# Attach the camera
sudo usbip attach -r $ip -b $busid
# --debug flag if errors


