
default: prepare | domotica_daemon

.PHONY: domotica_daemon

domotica_daemon:
	make -C domotica_daemon

prepare:
	git submodule init pcdev
	git submodule update
