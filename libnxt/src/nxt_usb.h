/*! \file
 * \brief Declares functions for basic USB operations to be performed on an NXT.
 *
 * The functions declared here are intended as wrappers to the
 * __libusb-1.0__ api, conveniently replacing the series of function calls
 * required to open and configure a device, for example, with a single call.
 * All functions return libusb_error codes; see the libusb-1.0 API documentation
 * at [this page](http://libusb.sourceforge.net/api-1.0/group__misc.html) for
 * more information on libusb_error codes.
 *
 * Usage
 * =====
 * The following code structure must be implemented when using __nxt_usb__.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #include nxt_usb.h // include the header.
 *
 *
 * int err = libusb_init( NULL ); // Initialise the libusb library.
 * if ( err ) {
 *     // To print the error name to stdout, uncomment next line.
 *     // printf( "Error: %s\n", libusb_error_name( err ) );
 *
 *     // Exit, because there is a problem with libusb.
 * }
 * // To enable debug messages to be generated by libusb, uncomment next line.
 * // libusb_set_debug( NULL, 3 );
 *
 * libusb_device * nxt = NULL;
 * err = find_device( &nxt );
 * // handle error
 * if ( err ) {
 *     // No need to call forget_nxt() here.
 * }
 *
 * libusb_device_handle * handle = NULL;
 * if ( nxt != NULL ) {
 *     err = open_device( nxt, &handle );
 *     // handle error
 *     if ( err ) {
 *         // If you will not try open_device() again, uncomment the next line.
 *         // forget_nxt( nxt );
 *
 *         // ...
 *     }
 * }
 *
 * // Application code including calls to bulk_read_nxt() and bulk_write_nxt().
 *
 * if ( handle != null ) {
 *     close_handle( handle );
 *     forget_nxt( nxt );
 * }
 *
 * libusb_exit( NULL );
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef NXT_USB_H
#define NXT_USB_H
#include <libusb.h>
#include <stdlib.h>

// The maximum USB bulk transfer data payload size supported by NXT.
#define MAX_PKT_SIZE 64

/*
 * For more information on the following, see Chapter 9 of the
 * USB Specification 2.0.
 */
// bConfigurationValue to use when configuring the NXT as part of open_nxt().
#define CONFIGURATION 1
// bInterfaceNumber to use when configuring the NXT as part of open_nxt().
#define INTERFACE 0
// bEndpointAddress for the Bulk Transfer IN endpoint.
#define BULK_READ_EP 0x82
// bEndpointAddress for the Bulk Transfer OUT endpoint.
#define BULK_WRITE_EP 0x01

/*! \brief Find an NXT device if it is physically connected to the Galileo.
 *
 * Get a reference to a logical USB device representing a Lego NXT
 * connected to the Galileo on the USB Host port. The device that is returned
 * should be later freed by calling `forget_nxt()`.
 * \param [out] nxt Output location for the returned `libusb_device` pointer.
 * Set to NULL when the return code is non-zero, indicating an error.
 * \return
 * \parblock
 * LIBUSB_SUCCESS if the NXT was found
 *
 * LIBUSB_ERROR_NOT_FOUND if the NXT is not connected to the Galileo
 *
 * another LIBUSB_ERROR_CODE on other failure.
 * \endparblock
 */
int find_nxt( libusb_device ** nxt );

/*! \brief Free a `libusb_device` previously obtained using `find_nxt()`.
 *
 * Call this function after `close_handle()`.
 * \param [in] nxt A `libusb_device` corresponding to the NXT.
 */
void forget_nxt( libusb_device * nxt );

/*! \brief Obtain a device handle that is required to perform I/O on an NXT.
 *
 * The handle that is returned must be passed to `bulk_read_nxt()` and
 * `bulk_write_nxt()`. To clean up after using the handle, call
 * `close_handle()` followed by `forget_nxt()`.
 *
 * \param [in] nxt A `libusb_device` corresponding to the NXT, returned by
 * `find_nxt()`.
 * \param [out] handle Output locaiton for the returned `libusb_device_handle`
 * pointer. Set to NULL when the return code is non-zero, indicating an error.
 * \return
 * \parblock
 * LIBUSB_SUCCESS
 *
 * LIBUSB_ERROR_NO_DEVICE if the NXT disconnected during the call
 *
 * another LIBUSB_ERROR_CODE on other failure.
 * \endparblock
 */
int open_nxt( libusb_device * nxt, libusb_device_handle ** handle );

/*! \brief Close a handle previously obtained using `open_nxt()`.
 *
 * Should be called on all open handles before your application exits.
 * \param [in] handle A `libusb_device_handle`.
 */
void close_handle( libusb_device_handle * handle );

/*! \brief Read data from the NXT using a bulk transfer pipe.
 *
 * Check the `transferred` parameter even if the return code indicates success, as
 * less data than expected may have been read. Also do not assume that timeout
 * conditions indicate a complete lack of I/O, check `transferred`.
 * \param [in] handle - A handle for the NXT to communicate with, previously
 * obtained using `open_nxt()`.
 * \param [out] buf - A buffer to read bytes of data into.
 * \param [in] offset The index in the buffer to store the first byte read.
 * \param [in] length The maximum number of bytes to read. The actual number of
 * bytes that are read could be less.
 * \param transferred The number of bytes successfully read from the NXT.
 * \return
 * \parblock
 * LIBUSB_SUCCESS (and populates `transferred`)
 *
 * LIBUSB_ERROR_TIMEOUT (and populates `transferred`)
 *
 * LIBUSB_ERROR_IO
 *
 * LIBUSB_ERROR_NO_DEVICE if the NXT disconnected
 *
 * another LIBUSB_ERROR_CODE on other failure.
 * \endparblock
 */
int bulk_read_nxt( libusb_device_handle * handle, unsigned char * buf,
                          size_t offset, size_t length, int * transferred );

/*! \brief Write data to the NXT using a bulk transfer pipe.
 *
 * Check the `transferred` parameter even if the return code indicates success,
 * as not all data may have been written. Also do not assume that timeout
 * conditions indicate a complete lack of I/O, check `transferred`.
 * \param [in] handle A handle for the NXT to communicate with, previously
 * obtained using `open_nxt()`.
 * \param [in] buf A buffer containing bytes of data to write.
 * \param [in] offset The index in the buffer of the first byte to write.
 * \param [in] length The number of bytes to write.
 *\param [out] transferred The number of bytes successfully written to the NXT.
 * \return
 * \parblock
 * LIBUSB_SUCCESS (and populates `transferred`)
 *
 * LIBUSB_ERROR_TIMEOUT (and populates `transferred`)
 *
 * LIBUSB_ERROR_IO
 *
 * LIBUSB_ERROR_NO_DEVICE if the NXT disconnected
 *
 * another LIBUSB_ERROR_CODE on other failure.
 * \endparblock
 */
int bulk_write_nxt( libusb_device_handle * handle, unsigned char * buf,
                           size_t offset, size_t length, int * transferred );

#endif