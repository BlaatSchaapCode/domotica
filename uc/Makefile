
default: usb_dongle ac_switch


prepare:
	cd ucdev && git submodule init ext/CMSIS_5
	cd ucdev && git submodule init ext/RTT
	cd ucdev && git submodule init ext/stm32/stm32f1xx_hal_driver
	cd ucdev && git submodule init ext/stm32/cmsis_device_f1
	cd ucdev && git submodule init ext/u8g2
	cd ucdev && git submodule init lib/bsradio
	cd ucdev && git submodule init lib/bssensor
	cd ucdev && git submodule init lib/libhalglue
	cd ucdev && git submodule init lib/libhlusbd
	cd ucdev && git submodule init lib/i2c_eeprom
	cd ucdev && git submodule update
	cd ucdev/lib/libhlusbd && git submodule init ConvertUTF && git submodule update
	ARCH=ARM SUBARCH=M3 DEBUG=1 make -C ucdev/build/staticlib/stm32f1
	ARCH=ARM SUBARCH=M3 RELEASE_SPEED=1 make -C ucdev/build/staticlib/stm32f1
	ARCH=ARM SUBARCH=M3 DEBUG=1 make -C ucdev/build/staticlib/rtt
	ARCH=ARM SUBARCH=M3 RELEASE_SPEED=1 make -C ucdev/build/staticlib/rtt
	ARCH=ARM SUBARCH=M3 DEBUG=1 make -C ucdev/build/staticlib/u8g2
	ARCH=ARM SUBARCH=M3 RELEASE_SPEED=1 make -C ucdev/build/staticlib/u8g2

