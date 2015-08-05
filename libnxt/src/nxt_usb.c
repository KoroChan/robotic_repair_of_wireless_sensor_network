#include "nxt_usb.h"

// The length of time to wait for an I/O function to return, in ms.
#define TIMEOUT 20000
// USB vendor ID for the Lego Company.
#define VENDOR_LEGO 0x0694
// Product ID for the NXT 2.0.
#define PRODUCT_NXT 0x0002

int find_nxt( libusb_device ** nxt ) {

    int errorCode;

    libusb_device ** list;
    // NULL here refers to the default libusb context.
    ssize_t count = libusb_get_device_list( NULL, &list );
    if ( count < 0 )
        // Count contains error code.
        return count;

    int found = 0;
    int error = 0;
    size_t i;
    libusb_device * currDev;
    struct libusb_device_descriptor desc;
    for ( i = 0; i < count && ! error && ! found; i++ ) {
        currDev = list[i];
        errorCode = libusb_get_device_descriptor( currDev, &desc );
        if ( ! errorCode ) {
            if ( desc.idVendor == VENDOR_LEGO &&
                 desc.idProduct == PRODUCT_NXT ) {
                    found = 1;
                    *nxt = libusb_ref_device( currDev );
            }
	    } else {
			error = 1;
		}
    }
    libusb_free_device_list( list, 1 );

    if ( found ) {
        return LIBUSB_SUCCESS;
    } else {
	    *nxt = NULL;
	    return ( errorCode ? errorCode : LIBUSB_ERROR_NOT_FOUND );
	}
}

void forget_nxt( libusb_device * nxt ) {
    libusb_unref_device( nxt );
}

int open_nxt( libusb_device * nxt, libusb_device_handle ** handle ) {

    int errorCode = libusb_open( nxt, handle );
    if ( errorCode ) {
        *handle = NULL;
        return errorCode;
    }

    int interfaceClaimed = 0;
    errorCode = libusb_set_configuration( *handle, CONFIGURATION );
    if ( ! errorCode ) {
        errorCode = libusb_claim_interface( *handle, INTERFACE );
        if ( ! errorCode ) {
            interfaceClaimed = 1;
            // Discard any data that NXT might initially send.
            int read = 0;
            int ioError = 0;
            unsigned char * buf;
            buf = ( unsigned char * ) calloc( MAX_PKT_SIZE, sizeof ( char ) );
            size_t i;
            do {
                ioError = libusb_bulk_transfer( *handle, BULK_READ_EP, buf,
                                                  MAX_PKT_SIZE, &read, 1000 );
            } while ( ! ioError && read > 0 );
            free( buf );
        }
    }

    if ( errorCode ) {
        if ( interfaceClaimed )
            libusb_release_interface( *handle, INTERFACE );
        libusb_close( *handle );
        *handle = NULL;
        return errorCode;
    } else {
        return LIBUSB_SUCCESS;
    }
}

void close_handle( libusb_device_handle * handle ) {
	// Discard any data that NXT might have left to send.
    int read = 0;
    int ioError = 0;
    unsigned char * buf;
    buf = ( unsigned char * ) calloc( MAX_PKT_SIZE, sizeof ( char ) );
    size_t i;
    do {
        ioError = libusb_bulk_transfer( handle, BULK_READ_EP, buf,
                                          MAX_PKT_SIZE, &read, 1000 );
    } while ( ! ioError && read > 0 );
    free( buf );
    libusb_release_interface( handle, INTERFACE );
    libusb_close( handle );
}

int bulk_write_nxt( libusb_device_handle * handle, unsigned char * buf,
                    size_t offset, size_t length, int * transferred ) {

    return libusb_bulk_transfer( handle, BULK_WRITE_EP, buf + offset, length,
                                 transferred, TIMEOUT );
}

int bulk_read_nxt( libusb_device_handle * handle, unsigned char * buf,
                   size_t offset, size_t length, int * transferred ) {

    return libusb_bulk_transfer( handle, BULK_READ_EP, buf + offset, length,
                                 transferred, TIMEOUT );
}
