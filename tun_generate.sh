#!/bin/bash

#Adding default route
ip route add default via 192.168.1.1 dev tun4

# Delete existing TUN interfaces
sudo ip link delete tun4
sudo ip link delete tun5

# Create new TUN interfaces
sudo ip tuntap add mode tun dev tun4
sudo ip tuntap add mode tun dev tun5

# Assign IP addresses to the TUN interfaces
sudo ip addr add 192.168.1.2/24 dev tun4
sudo ip addr add 192.168.2.2/24 dev tun5

# Bring up the TUN interfaces
sudo ip link set dev tun4 up
sudo ip link set dev tun5 up

# Display the status of TUN interfaces
sudo ip addr show