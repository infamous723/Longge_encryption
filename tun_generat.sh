#!/bin/bash

# Delete existing TUN interfaces
sudo ip link delete tun1
sudo ip link delete tun2

# Create new TUN interfaces
sudo ip tuntap add mode tun dev tun1
sudo ip tuntap add mode tun dev tun2

# Assign IP addresses to the TUN interfaces
sudo ip addr add 192.168.1.1/24 dev tun1
sudo ip addr add 192.168.2.1/24 dev tun2

# Bring up the TUN interfaces
sudo ip link set dev tun1 up
sudo ip link set dev tun2 up

# Display the status of TUN interfaces
sudo ip addr show