/*! \file
 * \brief Defines codes to be returned from functions to indicate the status of
 * their completion.
 *
 * Some of these codes are intended to hide those returned from different
 * libraries that may be used to implement `nxt_comm.h`; however, it is
 * advised to log the errors generated by an underlying library, as they often
 * reveal important information.
 */
#ifndef ERROR_CODES_H
#define ERROR_CODES_H

/*! \brief Error codes.
 *
 * Functions should return 0 on success, +1 to indicate no effect, or
 * another negative code on failure. You can call `libnxt_error_message()`
 * to retrieve a basic description of an error code.
 */
typedef enum libnxt_error {
    LIBNXT_NO_EFFECT = 1,/*!< A function call had no effect.
							  \showinitializer */
    LIBNXT_SUCCESS = 0,/*!< A function call completed successfully.
							\showinitializer */
    LIBNXT_TIMEOUT = -1,/*!< An IO operation timed out.
							 \showinitializer */
    LIBNXT_ILLEGAL_ARG = -2,/*!< Illegal argument(s) passed to a function.
								 \showinitializer */
    LIBNXT_NOT_VISIBLE = -3,/*!< NXT is not physically connected to Galileo.
								 \showinitializer */
    LIBNXT_DISCONNECTED = -4,/*!< NXT disconnected during a function call.
								  \showinitializer */
    LIBNXT_NOT_OPENED = -5, /*!< Communications not open when attempting I/O.
								 \showinitializer */
    LIBNXT_IO_ERROR = -6, /*!< An I/O error occurred.
							   \showinitializer */
    LIBNXT_DEPENDENT_ERROR = -7, /*!< Error returned by a dependent library.
									  \showinitializer */
    LIBNXT_OTHER_ERROR = -8 /*!< Other error.
								 \showinitializer*/
} libnxt_error;

/*! \brief Get a basic description of an error code.
 *
 * The caller must not `free()` the returned string.
 * \return A constant NULL-terminated string with an ASCII message related to
 * the given error code.
 */
const char * libnxt_error_message( libnxt_error errorCode );

#endif