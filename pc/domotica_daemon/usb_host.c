#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <libusb.h>

int main(int argc, char* argv[]) {
	int result;
	result = libusb_init(NULL);
	if (result) {
		puts("Cannot init");
		return -1;
	}
	libusb_device_handle *handle = libusb_open_device_with_vid_pid(NULL, 0xDEAD, 0xBEEF);
	if (!handle) {
		puts("Cannot open device");
		return -1;
	}
	if (libusb_claim_interface(handle,0)) {
		puts("Cannot claim interface");
		return -1;
	}
	
/*
int libusb_interrupt_transfer 	( 	libusb_device_handle *  	dev_handle,
		unsigned char  	endpoint,
		unsigned char *  	data,
		int  	length,
		int *  	transferred,
		unsigned int  	timeout 
	) 	
*/
	char buffer[256];
	
	while (true) {
		int transferred = 0;
		result = libusb_interrupt_transfer(handle, 0x81, buffer, sizeof(buffer),
			&transferred, 1000);
		if (!result) {
			if (transferred) {
				printf("Received %3d bytes\n",transferred);
				for (int i = 0 ; i < transferred; i++ ) {
					if (! ( i%4) ) printf ( "    ");
					if (! ( i%16) ) printf ( "\n");
					printf("%02X ", buffer[i]&0xFF);

				}
				printf ( "\n\n\n");
			}
		}
	}
}
