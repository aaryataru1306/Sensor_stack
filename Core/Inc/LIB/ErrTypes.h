#ifndef ERRTYPES_H
#define ERRTYPES_H

typedef enum {
    OK = 0,
    NOK,
    TIMEOUT_STATE,
    NULL_POINTER,
    INVALID_PARAM,
    BUS_ERROR,
    NOT_READY
} ErrorState_t;

#endif
