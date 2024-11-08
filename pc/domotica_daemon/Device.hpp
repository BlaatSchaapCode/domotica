/*
 * Device.hpp
 *
 *  Created on: 18 okt. 2019
 *      Author: andre
 */

#ifndef TESTLIBCODE_DEVICE_HPP_
#define TESTLIBCODE_DEVICE_HPP_

// Include libusb before C++ includes, otherwise Windows build fails
extern "C" {
#include <libusb.h>
}

#include <atomic>
#include <condition_variable>
#include <deque>
#include <stdint.h>
#include <string>
#include <thread>
#include <vector>
#include <shared_mutex>

#include "IDevice.hpp"

extern "C" {
// Internal library includes
#include "protocol.h"
#include "time_protocol.h"
}

class Device : public IDevice {
  public:
    Device(libusb_device_handle *handle);
    ~Device();

    uint32_t getSerial() { return m_serial; }
    libusb_device *getLibUsbDevice() { return m_usb_device; }


    int setTime(int node_id);
    int getData(int node_id);
    int setSwitch(int node_id, bool onoff);
    int getInfo(int node_id);


    int enqueuePacket(bscp_protocol_packet_t*);

  private:
    libusb_device *m_usb_device  = nullptr;
    libusb_device_handle *m_usb_handle = nullptr;

    struct libusb_device_descriptor m_usb_descriptor_device = {};
    unsigned char m_usb_string_manufacturer[256] = {};
    unsigned char m_usb_string_product[256] = {};
    unsigned char m_usb_string_serial[256] = {};
    uint32_t m_serial;


    //std::string m_serial = {};

    bool m_recv_queue_running = false;
    std::thread m_recv_queue_thread = {};
    std::condition_variable m_recv_queue_cv  = {};
    std::mutex m_recv_queue_mutex = {};
    static void process_recv_queue_code(Device *mc);


    bool m_send_queue_running = false;
    std::thread m_send_queue_thread = {};
    std::condition_variable m_send_queue_cv  = {};
    std::mutex m_send_queue_mutex = {};
    static void process_send_queue_code(Device *mc);




    struct libusb_transfer *m_transfer_in = nullptr;
    std::condition_variable m_transfer_cv = {};
    std::atomic<bool> m_transfer_pred = false;
    uint8_t m_recv_buffer[256]  = {};
    std::mutex m_transfer_mutex  = {};

    static void LIBUSB_CALL libusb_transfer_cb(struct libusb_transfer *transfer);

    std::deque<std::vector<uint8_t>> m_recv_queue  = {};


    std::deque<bscp_protocol_packet_t*> m_send_queue  = {};



};

#endif /* TESTLIBCODE_DEVICE_HPP_ */
