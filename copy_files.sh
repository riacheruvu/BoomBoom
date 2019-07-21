#!/bin/bash

# This script will copy a file from a docker container running on your robot, to your robot, and then to the folder of the system you're running this script from.
# This file is only if you are performing Docker development directly on your robot, and need to copy over files from the container to your system for viewing
#
# Usage: ./copy_files.sh
#
# The first time you run this file, the hostname, username, IP addr, container name, and filepath you enter are stored in a copy_files_config.conf file
# You can edit this file if any of this information changes for future runs.
# Everytime you run this program (even after the first run), you will always be prompted for the name of the file to copy over.



if [ -e copy_files_config.conf ]
then
    echo "Using config paramters from conf file"
    source copy_files_config.conf
else
    echo "Please enter the following details:"
    read -p "Enter Your Duckiebot's hostname: "  bothostname
    read -p "Enter Your Duckiebot's username: "  username 
    read -p "Enter Your Duckiebot's IP address: "  ipaddr
    read -p "Enter Your Duckiebot's container name: "  containername
    read -p "Enter Your file's filepath without a slash at the end: "  filepath
    echo "username=$username" >> copy_files_config.conf
    echo "bothostname=$bothostname" >> copy_files_config.conf
    echo "ipaddr=$ipaddr" >> copy_files_config.conf
    echo "filename=$containername" >> copy_files_config.conf
    echo "filename=$filepath" > copy_files_config.conf
fi

read -p "Enter the file name (including extension) of the file you'd like to transfer: "  filename
ssh $bothostname sudo docker cp $containername:$filepath/$filename  .
scp $username@$ipaddr:$filename . 

#end main()
