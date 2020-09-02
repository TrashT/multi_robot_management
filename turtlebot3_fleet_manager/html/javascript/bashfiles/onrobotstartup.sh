#!/bin/bash
# arguments: [network interface name, robot password, server IP]
ip4=$(/sbin/ip -o -4 addr list $1 | awk '{print $4}' | cut -d/ -f1)
suffix=$(echo $ip4 | cut -d'.' -f 4)
namespace=$(echo robot$suffix)

curl --data '{"name":"$USER@$ip4","host":"$ip4","user":"$USER","pass":"$2","model":"$TURTLEBOT3_MODEL"}' http://$3:8081/robot
