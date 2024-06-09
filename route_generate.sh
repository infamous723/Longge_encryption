# Enable IPv4 forwarding
sudo sysctl -w net.ipv4.ip_forward=1
sudo sysctl -p

# Add IP routes for tun1 and tun2
sudo ip route add 192.168.1.1 dev tun1
#sudo ip route add 192.168.2.1 dev eth0

# Routing process
# Choose one interface for MASQUERADE (e.g., eth0)
sudo iptables -t nat -A POSTROUTING -o tun1 -j MASQUERADE

# Forwarding rules
sudo iptables -A FORWARD -i tun1 -o tun1 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i tun1 -o tun1 -j ACCEPT

#sudo iptables -A FORWARD -i eth0 -o tun2 -m state --state RELATED,ESTABLISHED -j ACCEPT
#sudo iptables -A FORWARD -i tun2 -o eth0 -j ACCEPT