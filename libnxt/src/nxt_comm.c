#include "nxt_comm.h"
#include "nxt_usb.h"

static libusb_device_handle * handle = NULL;

libnxt_error open_comm( void ) {
    if ( handle != NULL )
        return LIBNXT_NO_EFFECT;

    int errorCode = libusb_init( NULL );
    if ( errorCode )
        return LIBNXT_DEPENDENT_ERROR;

    libusb_set_debug( NULL, 3 );

    libusb_device * nxt;
    errorCode = find_nxt( &nxt );
    if ( errorCode )
		return ( errorCode == LIBUSB_ERROR_NOT_FOUND ? LIBNXT_NOT_VISIBLE :
		                                               LIBNXT_DEPENDENT_ERROR );

    errorCode = open_nxt( nxt, &handle );
    if ( errorCode ) {
        forget_nxt( nxt );
        return ( errorCode == LIBUSB_ERROR_NO_DEVICE ? LIBNXT_DISCONNECTED :
                                                       LIBNXT_DEPENDENT_ERROR );
    }

    return LIBNXT_SUCCESS;
}

void close_comm( void ) {
    if ( handle != NULL ) {
		libusb_device * nxt = libusb_get_device( handle );
        close_handle( handle );
        forget_nxt( nxt );
        libusb_exit( NULL );
        handle = NULL;
    }
}

libnxt_error raw_read( unsigned char * buf, size_t offset, size_t maxLength,
                       int timeout, int * transferred ) {
    if ( handle == NULL )
        return LIBNXT_NOT_OPENED;

    if ( maxLength == 0 ) {
		*transferred = 0;
        return LIBNXT_NO_EFFECT;
	}

    int unfinished = 0;
    int errorCode;
    int total = 0;
    int read;
    int ioError = 0;
    int waitForData = ! timeout;
    do {
		read = 0;
        errorCode = bulk_read_nxt( handle, buf, offset + total,
                                   maxLength - total, &read );
        if ( errorCode && errorCode != LIBUSB_ERROR_TIMEOUT ) {
			ioError = 1;
		} else {
			total += read;
			if ( ! unfinished ) {
				unfinished = errorCode == LIBUSB_ERROR_TIMEOUT && read > 0;
			} else {
				if ( errorCode == LIBUSB_SUCCESS ) {
					unfinished = 0;
				}
			}
		}
    } while ( ( waitForData && ! ioError && read == 0 ) || unfinished );

    if ( ioError ) {
		return ( errorCode == LIBUSB_ERROR_NO_DEVICE ? LIBNXT_DISCONNECTED :
		                                               LIBNXT_IO_ERROR );
    } else {
		*transferred = total;
		return ( errorCode == LIBUSB_ERROR_TIMEOUT ? LIBNXT_TIMEOUT :
		                                             LIBNXT_SUCCESS );
    }
}

libnxt_error raw_write( unsigned char * buf, size_t offset, size_t length,
                        int timeout, int * transferred ) {
    if ( handle == NULL )
        return LIBNXT_NOT_OPENED;

    if ( length == 0 ) {
		*transferred = 0;
        return LIBNXT_NO_EFFECT;
	}

    int errorCode;
    int total = 0;
    int written;
    int ioError = 0;
    int waitForData = ! timeout;
    do {
		written = 0;
        errorCode = bulk_write_nxt( handle, buf, offset + total, length - total,
                                    &written );
        if ( errorCode && errorCode != LIBUSB_ERROR_TIMEOUT ) {
            ioError = 1;
        } else {
            total += written;
        }
    } while ( waitForData && ! ioError && total < length );

    if ( ioError ) {
		return ( errorCode == LIBUSB_ERROR_NO_DEVICE ? LIBNXT_DISCONNECTED :
		                                               LIBNXT_IO_ERROR );
    } else {
        *transferred = total;
        return ( errorCode == LIBUSB_ERROR_TIMEOUT ? LIBNXT_TIMEOUT :
                                                     LIBNXT_SUCCESS );
    }
}
