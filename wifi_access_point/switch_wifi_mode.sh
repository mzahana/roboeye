#!/bin/bash

sudo systemctl stop openvins_ov9281_mpu_system.service
sudo systemctl disable openvins_ov9281_mpu_system.service
if [ "$1" == "ap" ]; then
    sudo systemctl stop dnsmasq
    sudo systemctl stop hostapd
    sudo cp /etc/dhcpcd.conf.ap /etc/dhcpcd.conf
    sudo systemctl restart dhcpcd
    sudo systemctl unmask hostapd
    sudo systemctl enable hostapd
    sudo systemctl start dnsmasq
    sudo systemctl start hostapd
    echo "Switched to Access Point mode"
elif [ "$1" == "client" ]; then
    sudo systemctl stop dnsmasq
    sudo systemctl stop hostapd
    sudo cp /etc/dhcpcd.conf.client /etc/dhcpcd.conf
    sudo systemctl restart dhcpcd
    sudo systemctl disable hostapd
    echo "Switched to Client mode"
else
    echo "Usage: $0 {ap|client}"
fi
sudo systemctl enable openvins_ov9281_mpu_system.service
sudo systemctl start openvins_ov9281_mpu_system.service
sudo reboot
