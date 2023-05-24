#!/bin/bash

# Host my USB webcam over IP so my server running my ros nodes can grab it.

# Start the system service and the kernel modules
sudo modprobe usbip_core
sudo modprobe usbip_host
systemctl start usbipd.service

# Vendor ID for my Logitech Webcam
id="046d:0821" 

# Get the busID from the vendorID
busid=$(/usr/sbin/usbip list -p -l | grep -i "usbid=$id" | cut '-d#' -f1) 
echo $busid

# Bind the Webcam
usbip bind --$busid


