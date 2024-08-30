/*

 File: 		DeviceManager.cpp
 Author:	André van Schoubroeck
 License:	MIT


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

#include "DeviceManager.hpp"

#include <stdarg.h>
#include <string.h>

#include "threadname.hpp"

DeviceManager *DeviceManager::dm = nullptr;

// DeviceManager::DeviceManager() {
void DeviceManager::start() {
    auto version = libusb_get_version();
    printf("Using libusb version %d.%d.%d.%d", version->major, version->minor, version->micro, version->nano);
    if (version->rc) {
        if (strlen(version->rc)) {
            putchar('-');
            puts(version->rc);
        }
    }
    puts("\n");
    printf("Initialising libusb\n");

    int res = libusb_init(&ctx);

    if (res != 0) {
        fprintf(stderr, "Error initialising libusb.\n");
        return;
    }

    printf("Starting hotplug callback thread\n");
    libusb_hotplug_callback_thread = thread(libusb_hotplug_callback_thread_code, this);

    printf("Starting events thread\n");
    libusb_handle_events_thread = thread(libusb_handle_events_thread_code, this);

    printf("Registering hotplug callback\n");

    DeviceManager::dm = this;

    // todo allow for registering devices with their pid vid

    res = libusb_hotplug_register_callback(
        ctx, libusb_hotplug_event(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT), LIBUSB_HOTPLUG_ENUMERATE,
        0xDEAD, 0xBEEF, LIBUSB_HOTPLUG_MATCH_ANY, DeviceManager::libusb_hotplug_callback, this, &hotplug_handle);
}

DeviceManager::~DeviceManager() {

    printf("DeviceManager::~DeviceManager()\n");

    libusb_hotplug_deregister_callback(ctx, hotplug_handle);

    libusb_hotplug_callback_thread_running = false;
    libusb_hotplug_callback_cv.notify_all();
    if (libusb_hotplug_callback_thread.joinable())
        libusb_hotplug_callback_thread.join();

    for (auto const &[libusb_dev, bscp_dev] : mapUsb2Device) {
        delete bscp_dev;
    }

    libusb_handle_events_thread_running = false;
    if (libusb_handle_events_thread.joinable())
        libusb_handle_events_thread.join();

    libusb_exit(ctx);
}

void DeviceManager::libusb_handle_events_thread_code(DeviceManager *dm) {
    setThreadName("libusb_events");

    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    while (dm->libusb_handle_events_thread_running) {
        libusb_handle_events_timeout_completed(dm->ctx, &tv, nullptr);
    }
}

void DeviceManager::libusb_hotplug_callback_thread_code(DeviceManager *dm) {
    setThreadName("libusb_hotplug");
    while (dm->libusb_hotplug_callback_thread_running) {
        unique_lock<mutex> lk(dm->libusb_hotplug_callback_mutex);
        dm->libusb_hotplug_callback_cv.wait(lk);
        if (!dm->libusb_hotplug_callback_thread_running)
            return;

        while (!dm->libusb_hotplug_event_queue.empty()) {
            if (!dm->libusb_hotplug_callback_thread_running)
                return;
            auto libusb_hotplug_callback_event = dm->libusb_hotplug_event_queue.front();
            dm->libusb_hotplug_event_queue.pop();

            switch (libusb_hotplug_callback_event.event) {
            case LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED: {
                puts("LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED");
                libusb_device_handle *handle = NULL;
                int retval = libusb_open(libusb_hotplug_callback_event.dev, &handle);

                if (retval) {
                    printf("DeviceManager: Unable to open device: %s: %s\n", libusb_error_name(retval),
                           libusb_strerror((libusb_error)retval));
                    break;
                }

                Device *device = new Device(handle);

                dm->addController(dynamic_cast<Device *>(device));
                break;
            }
            case LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT: {
            	puts("LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT");
                auto controller = dm->mapUsb2Device[libusb_hotplug_callback_event.dev];
                if (controller) {
                    dm->eraseController(controller);
                }
                break;
            }
            default: {
                printf("Unhandled event %d\n", libusb_hotplug_callback_event.event);
            }
            }
        }
    }
}

int DeviceManager::libusb_hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev, libusb_hotplug_event event,
                                           void *user_data) {
    DeviceManager *dm = (DeviceManager *)(user_data);
    unique_lock<mutex> lk(dm->libusb_hotplug_callback_mutex);
    dm->libusb_hotplug_event_queue.push({ctx, dev, event});
    dm->libusb_hotplug_callback_cv.notify_all();
    return 0;
}

void DeviceManager::addController(Device *device) {
    auto serial = device->getSerial();

    mapSerial2Device[serial] = device;
    mapUsb2Device[device->getLibUsbDevice()] = device;
}

void DeviceManager::eraseController(Device *device) {
    mapSerial2Device.erase(device->getSerial());
    mapUsb2Device.erase(device->getLibUsbDevice());

    delete device;
}
