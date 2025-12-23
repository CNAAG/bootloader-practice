#ifndef PMBUS_CMDS_H
#define PMBUS_CMDS_H

#include <stdint.h>

/* QUERY */
#define PMBUS_QUERY_UNSUPPORTED_CMD     (0b00000000)
#define PMBUS_QUERY_WRITE_LINEAR        (0b11000000)
#define PMBUS_QUERY_WRITE_INT16         (0b11000100)
#define PMBUS_QUERY_WRITE_FLOAT         (0b11001000)
#define PMBUS_QUERY_WRITE_DIRECT        (0b11001100)
#define PMBUS_QUERY_WRITE_UINT8         (0b11010000)
#define PMBUS_QUERY_WRITE_VID           (0b11010100)
#define PMBUS_QUERY_WRITE_MFR_SPC       (0b11011000)
#define PMBUS_QUERY_WRITE_NOT_NUMERIC   (0b11011100)
#define PMBUS_QUERY_READ_LINEAR         (0b10100000)
#define PMBUS_QUERY_READ_INT16          (0b10100100)
#define PMBUS_QUERY_READ_FLOAT          (0b10101000)
#define PMBUS_QUERY_READ_DIRECT         (0b10101100)
#define PMBUS_QUERY_READ_UINT8          (0b10110000)
#define PMBUS_QUERY_READ_VID            (0b10110100)
#define PMBUS_QUERY_READ_MFR_SPC        (0b10111000)
#define PMBUS_QUERY_READ_NOT_NUMERIC    (0b10111100)

typedef struct PmbusCmdHandler {
    uint8_t cmd;
    uint8_t queryData;
    uint8_t numOfData;
    int (*handler)(const uint8_t* rxData, const uint8_t rwFlag);
} PMBUS_CMD_HANDLER;

uint8_t GetPmbusCmdDataLen(uint8_t cmd);


#endif /* PMBUS_CMDS_H */