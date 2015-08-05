#include "messaging.h"
#include "nxt_comm.h"

// Buffers can hold 512 bytes at a time.
#define BUFFER_SIZE 512
// LCP command type for command to enter packet transfer mode.
#define SYSTEM_COMMAND_REPLY 0x01
// System command to enter packet mode.
#define NXJ_PACKET_MODE 0xff
// Expected reply to request to enter packet mode.
static const unsigned char CONFIRM_PACKET_MODE_REPLY[] = { 0x02, 0xfe, 0xef };

static unsigned char * inBuf;

static unsigned char * outBuf;

// Number of bytes of data in inBuf.
static size_t inCount = 0;

/* Location in inBuf of the next byte to read. Bytes are never removed, but
 * overwritten by calls to fill_buffer().
 */
static size_t readOffset = 0;

// Number of bytes of data in outBuf
static size_t outCount = 0;

// Boolean flag for 20 second timeout on IO operations. TIMEOUT DISABLED.
static int timeout = 0;

// Boolean flag indicating whether the connection is open or closed.
static int eof = 1;

// EOF packet header: sent to indicate end of communication.
static unsigned char EOF_HEADER[] = { 0x00, 0x00 };

/*
 * return: A non-zero integer if inBuf is empty, otherwise 0.
 */
static int in_buffer_empty( void );

/*
 * Read bytes from the NXT into the inBuf until it is either full, the
 * read operation times out, or there is an error.
 * return: LIBNXT_SUCCESS, or
 *         LIBNXT_DISCONNECTED if the NXT disconnected during the call, or
 *         LIBNXT_IO_ERROR, or
 *         LIBNXT_TIMEOUT.
 */
static libnxt_error fill_buffer( void );

/*
 * return: A non-zero integer if outBuf is full, otherwise 0.
 */
static int out_buffer_full( void );

/*
 * Write bytes from the outBuf to the NXT until it is empty, the
 * write operation times out, or there is an error.
 * return: LIBNXT_SUCCESS, or
 *         LIBNXT_NO_EFFECT if the outBuf is already empty, or
 *         LIBNXT_DISCONNECTED if the NXT disconnected during the call, or
 *         LIBNXT_IO_ERROR, or
 *         LIBNXT_TIMEOUT.
 */
static libnxt_error flush_buffer( void );

/*
 * Fetch the next byte of data from the inBuf, filling the buffer if required.
 * out: byte - Location to store the data.
 * return: LIBNXT_SUCCESS, or
 *         LIBNXT_NO_EFFECT if there was no data available, or
 *         LIBNXT_DISCONNECTED if the NXT disconnected during the call, or
 *         LIBNXT_IO_ERROR.
 */
static libnxt_error read_byte( unsigned char * byte );

/*
 * Put a byte into the outBuf, flushing the buffer if required.
 * in: byte - Data to store in the buffer.
 * return: LIBNXT_SUCCESS, or
 *         LIBNXT_NO_EFFECT if the buffer could not be flushed, leaving no room
 *                          for the new data, or
 *         LIBNXT_DISCONNECTED if the NXT disconnected during the call, or
 *         LIBNXT_IO_ERROR.
 */
static libnxt_error write_byte( unsigned char byte );

/*
 * Send a special packet to the NXT to indicate the connection should close.
 * return: LIBNXT_SUCCESS, or
 *         LIBNXT_DISCONNECTED if the NXT disconnected during the call, or
 *         LIBNXT_IO_ERROR.
 */
static libnxt_error send_eof( void );

static int in_buffer_empty( void ) {
	return readOffset >= inCount;
}

static libnxt_error fill_buffer( void ) {
    inCount = 0;
    readOffset = 0;
    libnxt_error errorCode;
    errorCode = raw_read( inBuf, 0, BUFFER_SIZE, timeout, &inCount );
    return errorCode;
}

static int out_buffer_full( void ) {
	return outCount >= BUFFER_SIZE;
}

static libnxt_error flush_buffer( void ) {
    if ( outCount == 0 ) {
        return LIBNXT_NO_EFFECT;
    }
    int written = 0;
    libnxt_error errorCode;
    errorCode = raw_write( outBuf, 0, outCount, timeout, &written );
    outCount -= written;
    return errorCode;
}

static libnxt_error read_byte( unsigned char * byte ) {
    libnxt_error errorCode;
    int error = 0;
    if ( in_buffer_empty() ) {
        errorCode = fill_buffer();
        error = errorCode && errorCode != LIBNXT_TIMEOUT;
    }

    if ( ! error ) {
		if ( in_buffer_empty() ) {
		    return LIBNXT_TIMEOUT;
		} else {
		    *byte = inBuf[readOffset++];
            return LIBNXT_SUCCESS;
		}
    } else {
		return errorCode;
	}
}

static libnxt_error write_byte( unsigned char byte ) {
    libnxt_error errorCode;
    int error = 0;
    if ( out_buffer_full() ) {
        errorCode = flush_buffer();
        error = errorCode && errorCode != LIBNXT_TIMEOUT;
    }

    if ( ! error ) {
		if ( out_buffer_full() ) {
			return LIBNXT_TIMEOUT;
		} else {
            outBuf[outCount++] = byte;
            return LIBNXT_SUCCESS;
		}
    } else {
        return errorCode;
    }
}

static libnxt_error send_eof( void ) {
    libnxt_error errorCode;
    int sent = 0;
    // Always wait for EOF to be successfully sent.
    errorCode = raw_write( EOF_HEADER, 0, sizeof ( EOF_HEADER ), 0, &sent );
    return ( errorCode ? errorCode : LIBNXT_SUCCESS );
}

void set_timeout( int enabled ) {
    timeout = enabled;
}

libnxt_error init_messaging( void ) {
    if ( ! eof )
        return LIBNXT_NO_EFFECT;

    libnxt_error errorCode = open_comm();
    if ( errorCode )
        return errorCode;

    eof = 0;

    inBuf = (unsigned char *) calloc( BUFFER_SIZE, sizeof( char ) );
    outBuf = (unsigned char *) calloc( BUFFER_SIZE, sizeof( char ) );

    unsigned char request[] = { SYSTEM_COMMAND_REPLY, NXJ_PACKET_MODE };
    int transferred = 0;
    errorCode = raw_write( request, 0, sizeof ( request ), 0, &transferred );
    if ( ! errorCode ) {
        unsigned char reply[BUFFER_SIZE];
        errorCode = raw_read( reply, 0, sizeof ( reply ), 0, &transferred );
        if ( ! errorCode ) {
			int valid = 1;
            if ( transferred == sizeof ( CONFIRM_PACKET_MODE_REPLY ) ) {
                size_t i;
                for ( i = 0; i < transferred; i++ ) {
                    if ( reply[i] != CONFIRM_PACKET_MODE_REPLY[i] ) {
                        valid = 0;
                        break;
                    }
                }
            } else {
				valid = 0;
			}

            if ( ! valid ) {
                errorCode = LIBNXT_OTHER_ERROR;
			}
        }
    }

    if ( errorCode ) {
        close_comm();
        eof = 1;
        free( inBuf );
        free( outBuf );
        return errorCode;
    } else {
        return LIBNXT_SUCCESS;
    }
}

void exit_messaging( void ) {
    if ( ! eof ) {
		// Temporarily disable timeout if enabled.
		int tmpTimeout = timeout;
		set_timeout( 0 );
        if ( ! flush_buffer() ) {
            if ( ! send_eof() ) {
                unsigned char * dataIn;
                uint16_t length = 0;
                do {
                    if ( receive( &dataIn, &length ) )
                        break;
                } while ( length > 0 );
            }
        }
        close_comm();
        eof = 1;
        free( inBuf );
        free( outBuf );
        // Re-establish previous timeout setting.
        set_timeout( tmpTimeout );
    }
}

libnxt_error receive( unsigned char ** message, uint16_t * length ) {
    if ( eof )
        return LIBNXT_NOT_OPENED;

    libnxt_error errorCode;
    size_t i;

    char lenLSB, lenMSB;

    if ( ! ( errorCode = read_byte( &lenLSB ) ) )
        errorCode = read_byte( &lenMSB );

    if ( ! errorCode ) {
        *length = ( lenMSB << 8 ) | lenLSB;
        if ( *length == 0 ) {
            *message = REQUEST_EXIT;
        } else {
            unsigned char * ret;
            ret = (unsigned char *) calloc( *length, sizeof( char ) );
            for ( i = 0; ! errorCode && i < *length; i++ ) {
                errorCode = read_byte( &ret[i] );
            }
            if ( ! errorCode ) {
                *message = ret;
            }
        }
	}

    return ( errorCode ? errorCode : LIBNXT_SUCCESS );
}

void free_message( unsigned char * message ) {
    free( message );
}

libnxt_error send( unsigned char * message, uint16_t length ) {
    if ( eof )
        return LIBNXT_NOT_OPENED;

    libnxt_error errorCode;
    size_t i;

    errorCode = write_byte( (unsigned char) ( length & 0xFF ) ); // LSB
    if ( ! errorCode )
        write_byte( (unsigned char) ( length >> 8 ) ); // MSB

    if ( ! errorCode ) {
        for ( i = 0; ! ( errorCode || i >= length ); i++ ) {
            errorCode = write_byte( message[i] );
        }
    }

    return ( errorCode ? errorCode : flush_buffer() );
}
