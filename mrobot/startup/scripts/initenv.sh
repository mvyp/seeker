#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  mrobot"
echo "mrobot usb connection as /dev/mrobot , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy mrobot.rules to  /etc/udev/rules.d/"
echo "`rospack find startup`/scripts/mrobot.rules"
sudo cp `rospack find startup`/scripts/mrobot.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
