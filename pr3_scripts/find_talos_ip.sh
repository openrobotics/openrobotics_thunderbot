#!/bin/sh
# instructions: plug in computer to router. plug talos into same router. run script. ssh using the resulting ip address.
# It is okay if multiple ip addresses are printed. that means that both ethernet ports on talos are plugged into the same network
sudo nmap -sP 192.168.0.0-254 | grep -B 2 -i -E "40:61:86:04:8D:6(A|B)" | grep "Nmap scan report" | awk '{print "talos ip: " $5}'

