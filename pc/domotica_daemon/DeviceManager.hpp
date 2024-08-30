/*

 File: 		DeviceManager.hpp
 Author:	André van Schoubroeck
 License:	MIT

 SPDX-License-Identifier: MIT

 MIT License

 Copyright (c) 2017-2024 André van Schoubroeck <andre@blaatschaap.be>

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */
#ifndef SRC_DEVICEMANAGER_H_
#define SRC_DEVICEMANAGER_H_

// Include libusb before C++ includes, otherwise Windows build fails
extern "C" {
#include "libusb.h"
}

// C++ Standard Libraries
#include <condition_variable>
#include <map>
#include <queue> // std::queue
#include <string>
#include <thread> // std::thread
#include <vector>

#include "Device.hpp"

typedef struct {
	struct libusb_context *ctx;
	struct libusb_device *dev;
	libusb_hotplug_event event;
} libusb_hotplug_event_t;

#include "IDeviceManager.hpp"

class DeviceManager {
public:
	// DeviceManager();
	virtual ~DeviceManager();

	void start();

	Device* getDevice(std::string id) {
		return mapSerial2Device[id];
	}
private:
	libusb_context *ctx = nullptr;
	static DeviceManager *dm;

	void addController(Device *device);
	void eraseController(Device *device);

	bool libusb_hotplug_callback_thread_running = true;
	static void libusb_hotplug_callback_thread_code(DeviceManager *dm);
	std::thread libusb_hotplug_callback_thread = { };
	std::condition_variable libusb_hotplug_callback_cv = { };
	std::mutex libusb_hotplug_callback_mutex = { };

	libusb_hotplug_callback_handle hotplug_handle = { };

	static int LIBUSB_CALL libusb_hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev,
			libusb_hotplug_event event, void *user_data);

	std::queue<libusb_hotplug_event_t> libusb_hotplug_event_queue = { };

	std::thread libusb_handle_events_thread = { };
	bool libusb_handle_events_thread_running = true;
	static void libusb_handle_events_thread_code(DeviceManager *dm);

	std::map<std::string, Device*> mapSerial2Device = { };
	std::map<libusb_device*, Device*> mapUsb2Device = { };
};

#endif /* SRC_DEVICEMANAGER_H_ */
