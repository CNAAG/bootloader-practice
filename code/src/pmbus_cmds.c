#include "pmbus_cmds.h"

int Pmbus19hCmdCapability(const uint8_t* rxData, const uint8_t rwFlag)
{

}

int Pmbus1AhCmdQuery(const uint8_t* rxData, const uint8_t rwFlag);
int Pmbus98hCmdPmbusRevision(const uint8_t* rxData, const uint8_t rwFlag);
int Pmbus9AhCmdModelName(const uint8_t* rxData, const uint8_t rwFlag);
int PmbusD4hCmdMfrHWCompatibility(const uint8_t* rxData, const uint8_t rwFlag);
int PmbusD5hCmdMfrFWUploadCapability(const uint8_t* rxData, const uint8_t rwFlag);
int PmbusD6hCmdMfrFWUploadMode(const uint8_t* rxData, const uint8_t rwFlag);
int PmbusD7hCmdMfrUpload(const uint8_t* rxData, const uint8_t rwFlag);
int PmbusD8hCmdMfrFWUploadStatus(const uint8_t* rxData, const uint8_t rwFlag);
int PmbusD9hCmdFWRevision(const uint8_t* rxData, const uint8_t rwFlag);

static PMBUS_CMD_HANDLER PmbusCmdTable[] = {
    {0x19, PMBUS_QUERY_READ_NOT_NUMERIC, 1, Pmbus19hCmdCapability},
    {0x1A, PMBUS_QUERY_READ_NOT_NUMERIC, , Pmbus1AhCmdQuery},
    {0x98, PMBUS_QUERY_READ_NOT_NUMERIC, , Pmbus98hCmdPmbusRevision},
    {0x9A, PMBUS_QUERY_READ_NOT_NUMERIC, , Pmbus9AhCmdModelName},
    {0xD4, PMBUS_QUERY_READ_NOT_NUMERIC, , PmbusD4hCmdMfrHWCompatibility},
    {0xD5, PMBUS_QUERY_READ_NOT_NUMERIC, , PmbusD5hCmdMfrFWUploadCapability},
    {0xD6, PMBUS_QUERY_READ_NOT_NUMERIC, , PmbusD6hCmdMfrFWUploadMode},
    {0xD7, PMBUS_QUERY_READ_NOT_NUMERIC, , PmbusD7hCmdMfrUpload},
    {0xD8, PMBUS_QUERY_READ_NOT_NUMERIC, , PmbusD8hCmdMfrFWUploadStatus},
    {0xD9, PMBUS_QUERY_READ_NOT_NUMERIC, , PmbusD9hCmdFWRevision},
};


uint8_t GetPmbusCmdDataLen(uint8_t cmd)
{
    
}