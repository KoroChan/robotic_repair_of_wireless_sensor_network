/*! \file
 * \brief Functions to set up and perform packet-based communication with an
 * NXT running Lejos.
 *
 * The software of the Lejos NXJ project defines a simple packet structure with
 * a two-byte header that can be used in communications between an NXT and
 * another device over either USB or Bluetooth.
 * When using this packet-based communication, either device can indicate a
 * need to close the connection by sending a specially formatted _EOF_ packet.
 *
 * I/O functions are blocking.
 */
#ifndef MESSAGING_H
#define MESSAGING_H
#include "error_codes.h"
#include <stdint.h>

/*! \def REQUEST_EXIT
 * Check against messages returned by `receive()` if the NXT is requesting to
 * exit communications.
 */
#define REQUEST_EXIT NULL

/*! \brief Open communications with the NXT and perform handshake to
 * establish packet-based communication.
 *
 * Because this function involves I/O with the NXT, it may block.
 * \return
 * \parblock
 * \linkerror{LIBNXT_SUCCESS}
 *
 * \linkerror{LIBNXT_NO_EFFECT} if messaging has already been initialised
 *
 * \linkerror{LIBNXT_DEPENDENT_ERROR} if there was an error in
 * underlying I/O library
 *
 * \linkerror{LIBNXT_NOT_VISIBLE} if an NXT is not physically connected
 *
 * \linkerror{LIBNXT_DISCONNECTED} if the NXT disconnected during the call
 *
 * \linkerror{LIBNXT_IO_ERROR}.
 * \endparblock
 */
libnxt_error init_messaging( void );

/*! \brief Close communications with the NXT.
 *
 * Send the _EOF_ packet and wait to receive it in response before closing the
 * connection and freeing resources.
 *
 * Because this function involves I/O with the NXT, it may block.
 */
void exit_messaging( void );

/*! \brief Receive a message from the NXT.
 *
 * Messages are byte-arrays formed from data in packets received from the NXT,
 * stripped of the header. This function may block.
 * \param [out] message
 * \parblock
 * If `length` > 0: the message received from the NXT.
 *
 * If `length` = 0: `#REQUEST_EXIT`; `exit_messaging()` should be called.
 * \endparblock
 * \param [out] length Size of the received message in bytes. `length` will be
 * 0 when `#REQUEST_EXIT` is returned in `message`.
 * \return
 * \parblock
 * \linkerror{LIBNXT_SUCCESS} (and populate parameters)
 *
 * \linkerror{LIBNXT_NOT_OPENED} if messaging has not yet been initialised
 *
 * \linkerror{LIBNXT_DISCONNECTED} if the NXT disconnected during the call
 *
 * \linkerror{LIBNXT_IO_ERROR}.
 */
libnxt_error receive( unsigned char ** message, uint16_t * length );

/*! \brief Free the memory taken up by message when it is no longer required.
 *
 * Since no I/O is performed, this function will not block.
 * \param message A byte array returned by `receive()`.
 */
void free_message( unsigned char * message );

/*! \brief Send a message to the NXT.
 *
 * A header is prefixed to the array of bytes before sending. This function may
 * block.
 * \param [in] message The message to send.
 * \param [in] length Size of the message to send, in bytes.
 * \return
 * \parblock
 * \linkerror{LIBNXT_SUCCESS} (and populate parameters)
 *
 * \linkerror{LIBNXT_NOT_OPENED} if messaging has not yet been initialised
 *
 * \linkerror{LIBNXT_DISCONNECTED} if the NXT disconnected during the call
 *
 * \linkerror{LIBNXT_IO_ERROR}.
 */
libnxt_error send( unsigned char * message, uint16_t length );

#endif
