/*! \file
 * \brief Encapsulates a single communication link between the Galileo and an
 * NXT.
 *
 * Abstracts the medium of physical connection between the Galileo and an NXT,
 * allowing for implementations that use USB or bluethooth. Intended for
 * use when a single NXT is connected to the Galileo. All functions return
 * `#libnxt_error` codes.
 * codes.
 */

#ifndef NXT_COMM_H
#define NXT_COMM_H
#include "error_codes.h"
#include <stdlib.h>

/*!
 * \brief Open communications with an NXT if it is physically connected to the
 * Galileo.
 *
 * \return
 * \parblock
 * \linkerror{LIBNXT_SUCCESS}
 *
 * \linkerror{LIBNXT_NO_EFFECT} if communications were previously opened
 *
 * \linkerror{LIBNXT_DEPENDENT_ERROR} if there was an error in dependent library
 *
 * \linkerror{LIBNXT_NOT_VISIBLE} if the NXT is not physically connected
 *
 * \linkerror{LIBNXT_DISCONNECTED} if the NXT disconnected during the call.
 * \endparblock
 */
libnxt_error open_comm( void );

/*!
 * \brief Close communications with the NXT.
 *
 * Release resources and perform necessary clean-up.
 */
void close_comm( void );

/*! \brief Read bytes from the NXT with an optional timeout.
 *
 * Check the `transferred` parameter even if the return code indicates success,
 * as less data than expected may have been read. Also, do not assume that
 * timeout conditions indicate a complete lack of I/O, check `transferred`.
 * \param [out] buf A buffer to read bytes of data into.
 * \param [in] offset The index in the buffer to store the first byte read.
 * \param [in] maxLength The maximum number of bytes to read. The actual number
 * of bytes that are read could be less.
 * \param [in] timeout A boolean flag for a timeout.
 * \param [out] transferred The number of bytes successfully read.
 * \return
 * \parblock
 * \linkerror{LIBNXT_SUCCESS} (and populates `transferred`)
 *
 * \linkerror{LIBNXT_NO_EFFECT} if maxLength is 0 (and populates `transferred`)
 *
 * \linkerror{LIBNXT_DISCONNECTED} if the NXT disconnected during the call
 *
 * \linkerror{LIBNXT_IO_ERROR}
 *
 * \linkerror{LIBNXT_TIMEOUT} (and populates `transferred`).
 * \endparblock
 */
libnxt_error raw_read( unsigned char * buf, size_t offset,
                              size_t maxLength, int timeout, int * transferred );

/*! \brief Write bytes to the NXT with an optional timeout.
 *
 * Do not assume that timeout conditions indicate a complete lack of I/O, check
 * the `transferred` parameter.
 * \param [in] buf A buffer containing bytes of data to write.
 * \param [in] offset The index in the buffer of first byte to write.
 * \param [in] length The number of bytes to write.
 * \param [in] timeout A boolean flag for a timeout.
 * \param [out] transferred The number of bytes successfully written.
 * \return
 * \parblock
 * \linkerror{LIBNXT_SUCCESS} (and populates `transferred`)
 *
 * \linkerror{LIBNXT_NO_EFFECT} if length is 0 (and populates `transferred`)
 *
 * \linkerror{LIBNXT_DISCONNECTED} if the NXT disconnected during the call
 *
 * \linkerror{LIBNXT_IO_ERROR}
 *
 * \linkerror{LIBNXT_TIMEOUT} (and populates `transferred`).
 * \endparblock
 */
libnxt_error raw_write( unsigned char * buf, size_t offset,
                               size_t length, int timeout, int * transferred );

#endif
