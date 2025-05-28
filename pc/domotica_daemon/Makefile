TARGET=domotica_daemon

PCDEV_ROOT=../pcdev
UCDEV_ROOT=../../uc/ucdev

OUT_DIR      = out


LIBS += libusb-1.0 libmosquittopp nlohmann_json sqlite3

CXX_SRC 	 += main.cpp
CXX_SRC 	 += Device.cpp
CXX_SRC 	 += DeviceManager.cpp
CXX_SRC 	 += SensorManager.cpp
CXX_SRC 	 += threadname.cpp
CXX_SRC 	 += mqtt.cpp
CXX_SRC 	 += sensordata.cpp
CXX_SRC 	 += fix.cpp

CXX_SRC 	 += utils/logger.cpp
CXX_SRC 	 += utils/time.cpp

CXX_INCLUDES += $(UCDEV_ROOT)/lib/bsprot

C_SRC		+= $(UCDEV_ROOT)/lib/bsprot/protocol.c


include $(PCDEV_ROOT)/build/make/all.mk

# eclipse
all: default
