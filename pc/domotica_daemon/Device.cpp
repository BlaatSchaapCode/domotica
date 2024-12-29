/*

 File: 		Device.cpp
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

// Matching header
#include "Device.hpp"

extern "C" {
// C library includes
#include <stdarg.h>
#include <string.h>
}
// C++ library includes
#include <chrono>
#include <deque>
#include <thread>

// External library includes
#include <libusb.h>

// Project includes
#include "threadname.hpp"

#include "SensorManager.hpp"
extern SensorManager g_sm;

Device::Device(libusb_device_handle *handle) {
	this->m_usb_device = libusb_get_device(handle);
	this->m_usb_handle = handle;
	int retval;

	retval = libusb_get_device_descriptor(m_usb_device,
			&m_usb_descriptor_device);
	if (retval) {
		puts("Error retrieving device descriptor");
	}

	retval = libusb_get_string_descriptor_ascii(m_usb_handle,
			m_usb_descriptor_device.iManufacturer, m_usb_string_manufacturer,
			sizeof(m_usb_string_manufacturer));

	retval = libusb_get_string_descriptor_ascii(m_usb_handle,
			m_usb_descriptor_device.iProduct, m_usb_string_product,
			sizeof(m_usb_string_product));

	retval = libusb_get_string_descriptor_ascii(m_usb_handle,
			m_usb_descriptor_device.iSerialNumber, m_usb_string_serial,
			sizeof(m_usb_string_serial));

	sscanf((const char*) m_usb_string_serial, "%08X", &m_serial);

	retval = libusb_claim_interface(m_usb_handle, 0);

	if (retval) {
		puts("Error claiming interface");
		return;
	}

	m_recv_queue_running = true;
	m_recv_queue_thread = std::thread(process_recv_queue_code, this);

	m_send_queue_running = true;
	m_send_queue_thread = std::thread(process_send_queue_code, this);

	m_transfer_in = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(m_transfer_in, handle, 0x81, m_recv_buffer,
			sizeof(m_recv_buffer), Device::libusb_transfer_cb, this, 50000);
	retval = libusb_submit_transfer(m_transfer_in);

	g_sm.dongleArrived(m_serial);
}

Device::~Device() {

	m_recv_queue_running = false;
	m_recv_queue_cv.notify_all();
	if (m_recv_queue_thread.joinable())
		m_recv_queue_thread.join();

	if (m_transfer_in) {
		int status = libusb_cancel_transfer(m_transfer_in);
		// If the transfer were still running, we have to cancel it here.
		// Note, it is expected to error before we reach this point,
		// At least it does on a Linux machine. This code is here just
		// in case.
		if (!status) {
			mutex m;
			unique_lock<mutex> lk(m);
			auto result = m_transfer_cv.wait_for(lk, 1s, [this] {
				return m_transfer_pred.load();
			});
			m_transfer_pred = false;
			if (result == false) {
				// The transfer has a pointer to this class instance. When
				// it gets destroyed, it will result in a use-after-free error.
				// Therefore, we cancel the transfer here in the destructor.
				// However, when it fails to cancel within 1 sec, it might
				// still do so later. This is not expected to happen, but
				// when it does, we'll probably crash!
				puts(
						"[ERROR] Timeout canceling transfer. We might crash later.");
			}
		} else {
			// We expect this to happen
			//				puts("Error cancelling transfer.");
		}
		libusb_free_transfer(m_transfer_in);
		m_transfer_in = nullptr;
	}

	g_sm.dongleLeft(m_serial);
}

void Device::libusb_transfer_cb(struct libusb_transfer *xfr) {
	Device *_this = (Device*) (xfr->user_data);

	if (!_this->m_recv_queue_running) {
		printf("Processing queue not running, shutdown in progress?\n");
		if (xfr->status == LIBUSB_TRANSFER_CANCELLED) {
			if (xfr == _this->m_transfer_in) {
				_this->m_transfer_pred = true;
				_this->m_transfer_cv.notify_all();
			}
		}
		return;
	}

	switch (xfr->status) {
	case LIBUSB_TRANSFER_COMPLETED:

		// Success here, data transfered are inside
		// transfer->buffer
		// and the length is
		// transfer->actual_length

		if (xfr->endpoint & 0x80) {
			std::vector<uint8_t> recvData;

			if (xfr->actual_length) {
				//recvData.resize(1 + xfr->actual_length);
				recvData.resize(4 + xfr->actual_length, 0);
				memcpy(4 + recvData.data(), xfr->buffer, xfr->actual_length);
				//recvData[0] = xfr->endpoint;
				(*(uint32_t*) &recvData[0]) = _this->m_serial;
				std::unique_lock<std::mutex> lk(_this->m_recv_queue_mutex);
				_this->m_recv_queue.push_back(recvData);
				_this->m_recv_queue_cv.notify_all();
			}
			int r;

			std::unique_lock<std::mutex> transfer_lock(_this->m_transfer_mutex);
			if (!_this->m_recv_queue_running)
				break;
			r = libusb_submit_transfer(xfr);

			if (r) {
				printf("Transfer Complete: Re-issue xfr error %s %s\n",
						libusb_error_name(r),
						libusb_strerror((libusb_error) r));
			}

		} else {

			// Free buffer
			free(xfr->buffer);
			// Free xfr
			libusb_free_transfer(xfr);
		}

		break;
	case LIBUSB_TRANSFER_OVERFLOW:
	case LIBUSB_TRANSFER_TIMED_OUT:



		if (xfr->endpoint & 0x80) {
			const char *msg =
					(xfr->status == LIBUSB_TRANSFER_OVERFLOW) ?
							"Overflow" : "Timeout";

			int r;
			std::unique_lock<std::mutex> transfer_lock(_this->m_transfer_mutex);
			if (!_this->m_recv_queue_running)
				break;

			r = libusb_submit_transfer(xfr);

			if (r) {
				printf("Transfer %s: Re-issue xfr error %s %s\n", msg,
						libusb_error_name(r),
						libusb_strerror((libusb_error) r));
			}
		} else {
			const char *msg =
					(xfr->status == LIBUSB_TRANSFER_OVERFLOW) ?
							"Overflow" : "Timeout";
			printf("Transmit %s\n", msg);

			// Free buffer
			free(xfr->buffer);
			// Free xfr
			libusb_free_transfer(xfr);
		}

		break;

	case LIBUSB_TRANSFER_ERROR:
		if (xfr->endpoint & 0x80) {
			printf("Transfer error while receiving on EP %02X\n",
					xfr->endpoint);
		} else {
			printf("Transfer error while transmitting on EP %02X\n",
					xfr->endpoint);
			libusb_free_transfer(xfr);
		}
		break;
	case LIBUSB_TRANSFER_STALL:
		printf("LIBUSB_TRANSFER_STALL\n");
		if (xfr->endpoint & 0x80) {
			printf("Transfer stalled while receiving on EP %02X\n",
					xfr->endpoint);
		} else {
			printf("Transfer stalled while transmitting on EP %02X\n",
					xfr->endpoint);
			libusb_free_transfer(xfr);
		}
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		printf("LIBUSB_TRANSFER_NO_DEVICE\n");
		if (xfr->endpoint & 0x80) {
			printf("No device while receiving on EP %02X\n", xfr->endpoint);
		} else {
			printf("No device while transmitting on EP %02X\n", xfr->endpoint);
			libusb_free_transfer(xfr);
		}
		break;
	case LIBUSB_TRANSFER_CANCELLED:
		if (xfr->endpoint & 0x80) {
			printf("Transfer cancelled while receiving on EP %02X\n",
					xfr->endpoint);

			if (xfr == _this->m_transfer_in) {
				_this->m_transfer_pred = true;
				_this->m_transfer_cv.notify_all();
			}

		} else {
			printf("Transfer cancelled while transmitting on EP %02X\n",
					xfr->endpoint);
			libusb_free_transfer(xfr);
		}
		break;
	}
}



void Device::process_send_queue_code(Device *dev) {
	setThreadName(std::string((char*) dev->m_usb_string_serial) + "_send");

	while (dev->m_send_queue_running) {
		unique_lock<mutex> lk(dev->m_send_queue_mutex);
		dev->m_send_queue_cv.wait(lk);
		if (!dev->m_send_queue_running)
			return;

		while (dev->m_send_queue.size()) {
			if (!dev->m_send_queue_running)
				return;

			auto packet = dev->m_send_queue.front();

			printf(
					"Message Sending Queue Sending packet SIZE: %3d, CMD: %02X\n",
					packet->head.size, packet->head.cmd);

			libusb_transfer *t = libusb_alloc_transfer(0);
			if (t) {
				dev->m_local_response_pred = false;
				dev->m_local_response_status = -1;
				dev->m_remote_response_pred = false;

				libusb_fill_bulk_transfer(t, dev->m_usb_handle, 0x01,
						(uint8_t*) packet, packet->head.size,
						Device::libusb_transfer_cb, dev, 50000);
				auto result = libusb_submit_transfer(t);
				printf("Result %d\n", result);
			}

			dev->m_send_queue.pop_front();

//			// Fixed sleep for now.
//			// TODO: Wait for response, and if forwarding packet success
//			// wait for additional message.
//			// this_thread::sleep_for(500ms);
//			this_thread::sleep_for(1s);


			// TODO: find the forward handler and signal the condition variables
			// used below, (initially without checked node ids)
			// It is currenty in main.cpp outside of a class, as the protocol
			// handler is currently C.
			// Should I make a C++ variant of the handler to make this easier?

			// Wait for the confirmation from the dongle (local confirmation)
			// the message has been transmitted successfully
			puts("Waiting for local confirmation");
			if (true) {
				unique_lock<mutex> lk(dev->m_local_response_mutex);
				auto result = dev->m_local_response_cv.wait_for(lk, 1s, [dev] {
				return dev->m_local_response_pred.load();
			});
			}
			puts("Received local confirmation");

			// If the message has been transmitted successfully, the node (remote)
			// will send a response. Wait for that response here

			if (!dev->m_local_response_status) {
				puts("Waiting for remote confirmation");
				unique_lock<mutex> lk(dev->m_remote_response_mutex);
				auto result = dev->m_remote_response_cv.wait_for(lk, 1s, [dev] {
					return dev->m_remote_response_pred.load();
				});
				puts("Received remote confirmation");
			} else {
				puts ("Local confirmation denotes error");
			}


		}
	}
}

void Device::process_recv_queue_code(Device *dev) {
	setThreadName(std::string((char*) dev->m_usb_string_serial) + "_Recv");

	while (dev->m_recv_queue_running) {
		unique_lock<mutex> lk(dev->m_recv_queue_mutex);
		dev->m_recv_queue_cv.wait(lk);
		if (!dev->m_recv_queue_running)
			return;

		while (dev->m_recv_queue.size()) {
			if (!dev->m_recv_queue_running)
				return;

			std::vector<uint8_t> data = dev->m_recv_queue.front();

			size_t size = data.size() - 4;
			uint8_t *buffer = data.data() + 4;
			uint32_t dongle_id = *(uint32_t*) (data.data());
			(void) size;

			protocol_parse(buffer, size, PROTOCOL_TRANSPORT_USB, &dongle_id);

			dev->m_recv_queue.pop_front();
		}
	}
}

int Device::setTime(int node_id) {
	bscp_protocol_packet_t *packet = (bscp_protocol_packet_t*) malloc(256);
	memset(packet, 0, 256);
	if (packet) {
		packet->head.cmd = BSCP_CMD_FORWARD;
		packet->head.sub = BSCP_SUB_QSET;
		bscp_protocol_forward_t *forward =
				(bscp_protocol_forward_t*) (packet->data);
		//		forward->head.to = 0x03;
		forward->head.to = node_id;
		forward->head.transport = PROTOCOL_TRANSPORT_RF;
		bscp_protocol_packet_t *forwarded_packet =
				(bscp_protocol_packet_t*) (forward->data);
		forwarded_packet->head.size = sizeof(bscp_protocol_header_t)
						+ sizeof(uint32_t);
		forwarded_packet->head.cmd = BSCP_CMD_UNIXTIME;
		forwarded_packet->head.sub = BSCP_SUB_QSET;
		*(uint32_t*) forwarded_packet->data = time(NULL);
		packet->head.size = forwarded_packet->head.size
				+ sizeof(bscp_protocol_forward_t)
				+ sizeof(bscp_protocol_header_t) + sizeof(uint32_t);

		return enqueuePacket(packet);
	}
	return -1;
}

int Device::getData(int node_id) {
	bscp_protocol_packet_t *packet = (bscp_protocol_packet_t*) malloc(256);
	memset(packet, 0, 256);
	if (packet) {
		packet->head.cmd = BSCP_CMD_FORWARD;
		packet->head.sub = BSCP_SUB_QSET;
		bscp_protocol_forward_t *forward =
				(bscp_protocol_forward_t*) (packet->data);
		//		forward->head.to = 0x03;
		forward->head.to = node_id;
		forward->head.transport = PROTOCOL_TRANSPORT_RF;
		bscp_protocol_packet_t *forwarded_packet =
				(bscp_protocol_packet_t*) (forward->data);
		forwarded_packet->head.size = sizeof(bscp_protocol_header_t);
		forwarded_packet->head.cmd = 0x10;
		forwarded_packet->head.sub = BSCP_SUB_QGET;
		packet->head.size = forwarded_packet->head.size
				+ sizeof(bscp_protocol_forward_t)
				+ sizeof(bscp_protocol_header_t);

		return enqueuePacket(packet);

	}
	return -1;

}

//--
int Device::getInfo(int node_id) {
	bscp_protocol_packet_t *packet = (bscp_protocol_packet_t*) malloc(256);
	memset(packet, 0, 256);
	if (packet) {
		packet->head.cmd = BSCP_CMD_FORWARD;
		packet->head.sub = BSCP_SUB_QSET;
		bscp_protocol_forward_t *forward =
				(bscp_protocol_forward_t*) (packet->data);
		//		forward->head.to = 0x03;
		forward->head.to = node_id;
		forward->head.transport = PROTOCOL_TRANSPORT_RF;
		bscp_protocol_packet_t *forwarded_packet =
				(bscp_protocol_packet_t*) (forward->data);
		forwarded_packet->head.size = sizeof(bscp_protocol_header_t);
		forwarded_packet->head.cmd = BSCP_CMD_INFO;
		forwarded_packet->head.sub = BSCP_SUB_QGET;
		packet->head.size = forwarded_packet->head.size
				+ sizeof(bscp_protocol_forward_t)
				+ sizeof(bscp_protocol_header_t);

		return enqueuePacket(packet);
	}
	return -1;

}
//--

int Device::setSwitch(int node_id, bool onoff) {
	bscp_protocol_packet_t *packet = (bscp_protocol_packet_t*) malloc(256);
	memset(packet, 0, 256);
	if (packet) {
		packet->head.cmd = BSCP_CMD_FORWARD;
		packet->head.sub = BSCP_SUB_QSET;
		bscp_protocol_forward_t *forward =
				(bscp_protocol_forward_t*) (packet->data);
		//		forward->head.to = 0x03;
		forward->head.to = node_id;
		forward->head.transport = PROTOCOL_TRANSPORT_RF;
		bscp_protocol_packet_t *forwarded_packet =
				(bscp_protocol_packet_t*) (forward->data);
		forwarded_packet->head.size = sizeof(bscp_protocol_header_t) + 1;
		forwarded_packet->head.cmd = 0x20;
		forwarded_packet->head.sub = BSCP_SUB_QSET;
		forwarded_packet->data[0] = onoff;
		packet->head.size = forwarded_packet->head.size
				+ sizeof(bscp_protocol_forward_t)
				+ sizeof(bscp_protocol_header_t);
		return enqueuePacket(packet);

	}
	return -1;

	return 0;
}

int Device::enqueuePacket(bscp_protocol_packet_t *packet) {
	printf("Enqueuing Send Packet Size %3d, CMD%02X\n",packet->head.size, packet->head.cmd);
	m_send_queue.push_back(packet);
	m_send_queue_cv.notify_all();
	return 0;
}

void Device::notify_local_response(uint8_t status){
	printf("Local Confirmation %02X\n", status);
	m_local_response_status = status;
	m_local_response_pred = true;
	m_local_response_cv.notify_all();
}

void Device::notify_remote_response(){
	puts("Remote Confirmation");
	m_remote_response_pred = true;
	m_remote_response_cv.notify_all();
}
