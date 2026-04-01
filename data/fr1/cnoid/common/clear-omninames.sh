#!/bin/bash

sudo /etc/init.d/omniorb4-nameserver stop
sudo rm /var/log/omniorb-nameserver.log 2> /dev/null
sudo rm /var/lib/omniorb/* 2> /dev/null
sudo rm -f /tmp/rtcmanager.ref
sudo /etc/init.d/omniorb4-nameserver start
sleep 1
tail /var/log/omniorb-nameserver.log
