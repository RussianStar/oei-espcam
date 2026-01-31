config:
	. /root/esp/esp-idf/export.sh
	idf.py -C /root/privat/garten/esp32s3-usb-snapper menuconfig
flash:
	ansible-playbook -i ansible/hosts.ini ansible/raspi-esp32-setup.yml
