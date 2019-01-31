if [ ! -e StimFolder ]
then
	mkdir StimFolder
fi
PICKPATH=$(pwd)
for i in $(ls)
do
	if [ -d $i ]
	then
		cd $i
		if [ -d slm_files ]
		then
			cd slm_files
			if [ -e spi_stim.txt ]
			then
				cp spi_stim.txt $PICKPATH/StimFolder/{$i}Stim.txt
			fi
			cd ../
		fi	
		cd ../
	fi
done
cd ../../../../fpga/sw/apps/spiload/	
make
cp spiload $PICKPATH/StimFolder
exit
