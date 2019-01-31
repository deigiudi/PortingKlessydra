#!/bin/bash

FOLDER=$1
FPGA_HOSTNAME=$2
TIMEOUT=$3

cd $1
scp spiload root@$FPGA_HOSTNAME:/root/
for FILE in $(ls)
do
	scp $FILE root@$FPGA_HOSTNAME:/root/
	ssh -t root@$FPGA_HOSTNAME /root/spiload --timeout=$TIMEOUT /root/$FILE
done
exit 0
