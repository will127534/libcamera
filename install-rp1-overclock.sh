dtc -@ -I dts -O dtb -o rp1-300mhz.dtbo rp1-300mhz.dtso
cp ./rp1-300mhz.dtbo /boot/firmware/overlays/
echo "dtoverlay=rp1-300mhz" >> /boot/firmware/config.txt
