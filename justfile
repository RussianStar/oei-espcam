config:
	bash -c '. /root/esp/esp-idf/export.sh && idf.py -C /root/privat/garten/esp32s3-usb-snapper menuconfig'

config-esp32cam:
	bash -c '. /root/esp/esp-idf/export.sh && idf.py -C /root/privat/garten/esp32s3-usb-snapper -B /root/privat/garten/esp32s3-usb-snapper/build-esp32cam -DIDF_TARGET=esp32 -DSDKCONFIG=sdkconfig.esp32cam -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32cam menuconfig'

build-esp32cam:
	bash -c '. /root/esp/esp-idf/export.sh && idf.py -C /root/privat/garten/esp32s3-usb-snapper -B /root/privat/garten/esp32s3-usb-snapper/build-esp32cam -DIDF_TARGET=esp32 -DSDKCONFIG=sdkconfig.esp32cam -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32cam build'

flash-esp32cam port="/dev/ttyUSB0":
	bash -c '. /root/esp/esp-idf/export.sh && idf.py -C /root/privat/garten/esp32s3-usb-snapper -B /root/privat/garten/esp32s3-usb-snapper/build-esp32cam -DIDF_TARGET=esp32 -DSDKCONFIG=sdkconfig.esp32cam -DSDKCONFIG_DEFAULTS=sdkconfig.defaults.esp32cam -p {{port}} flash'

flash:
	ansible-playbook -i ansible/hosts.ini ansible/raspi-esp32-setup.yml
