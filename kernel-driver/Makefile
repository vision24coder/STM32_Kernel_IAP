obj-m += stm32_ctrl.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

usb:
	sudo modprobe usbip-core
	sudo modprobe vhci-hcd
	sudo usbip attach -r 192.168.56.1 -b 1-10
	sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
	sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1
	sudo stty -F /dev/ttyACM0 115200 raw
	sudo fuser -k /dev/ttyACM0 || true
	sudo stty -F /dev/ttyACM0 115200 sane -echo hupcl
	
usb_reset:
		sudo stty -F /dev/ttyACM0 flush
	