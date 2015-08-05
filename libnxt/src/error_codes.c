#include "error_codes.h"

/* Error Messages */
static const char * NO_EFFECT_MSG = "An operation had no effect";
static const char * SUCCESS_MSG = "No errors occurred";
static const char * TIMEOUT_MSG = "An IO operation timed out";
static const char * ILLEGAL_ARG_MSG = "Illegal argument supplied to a function";
static const char * NOT_VISIBLE_MSG = "NXT is not physically connected to host";
static const char * DISCONNECTED_MSG = "NXT has become disconnected";
static const char * NOT_OPENED_MSG = "IO attempt when connection was not open";
static const char * IO_ERROR_MSG = "An error occurred during an IO operation";
static const char * DEPENDENT_ERROR_MSG = "Error in dependent library";
static const char * OTHER_ERROR_MSG = "An error occurred";
/* ************** */

const char * libnxt_error_message( libnxt_error errorCode ) {
    switch ( errorCode ) {
        case LIBNXT_NO_EFFECT:
            return NO_EFFECT_MSG;
        case LIBNXT_SUCCESS:
            return SUCCESS_MSG;
        case LIBNXT_TIMEOUT:
            return TIMEOUT_MSG;
        case LIBNXT_ILLEGAL_ARG:
            return ILLEGAL_ARG_MSG;
        case LIBNXT_NOT_VISIBLE:
            return NOT_VISIBLE_MSG;
        case LIBNXT_DISCONNECTED:
            return DISCONNECTED_MSG;
        case LIBNXT_NOT_OPENED:
            return NOT_OPENED_MSG;
        case LIBNXT_IO_ERROR:
            return IO_ERROR_MSG;
        case LIBNXT_DEPENDENT_ERROR:
            return DEPENDENT_ERROR_MSG;
        default:
            return OTHER_ERROR_MSG;
    }
}
