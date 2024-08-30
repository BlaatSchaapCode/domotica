TARGET=domotica_daemon

PCDEV_ROOT=../pcdev
UCDEV_ROOT=../../uc/ucdev

LIBS += libusb-1.0 libmosquittopp nlohmann_json

CXX_SRC 	 += main.cpp
CXX_SRC 	 += Device.cpp
CXX_SRC 	 += DeviceManager.cpp
CXX_SRC 	 += threadname.cpp
CXX_SRC 	 += mqtt.cpp

CXX_INCLUDES += $(UCDEV_ROOT)/lib/bsprot

C_SRC		+= $(UCDEV_ROOT)/lib/bsprot/protocol.c


include $(PCDEV_ROOT)/build/make/all.mk

# eclipse
all: default
