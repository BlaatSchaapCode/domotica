
default: prepare | all

.PHONY: prepare all

all:
	make -C uc
	make -C pc

prepare:
	make -C uc prepare
	make -C pc prepare 
