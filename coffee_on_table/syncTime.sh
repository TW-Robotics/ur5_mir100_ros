#!/bin/bash

echo "Syncing time with MiR-Robot at IP 192.168.12.20..."

echo "Syncing time..."
/etc/init.d/chrony stop
sudo ntpdate 192.168.12.20
/etc/init.d/chrony start

echo "New time difference to MiR:"
ntpdate -q 192.168.12.20