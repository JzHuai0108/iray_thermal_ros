#pragma once
#include <stdint.h>

#define OPEN_PORT_FAIL 201 //Failed to open the serial port
#define GET_COM_STATA_FAIL 202 //Failed to fetch serial port parameters 
#define SET_COM_STATA_FAIL 203 //Failed to set serial port parameters
#define SET_COM_TIMEOUT_FAIL 204 //Failed to set serial port timeout
#define SEND_DATA_FAIL 205 //Failed to send data 
#define RECV_DATA_FAIL 206 //Failed to receive data 
#define CLOSE_PORT_FAIL 207 //Failed to close the serial port
#define SEND_DATA_TIMEOUT 208 //Send timeout 
#define RECV_DATA_TIMEOUT 209 //Receive timeout 
#define MAX_WAIT_RESPONSE_TIME   60
#define MAX_LOCAL_BUF_LEN		512

#define  ERR_QUERY_INTERFACE_READ_FAIL 210   //get zoom fail
#define  ERR_QUERY_INTERFACE_WRITE_FAIL 211   //set zoom fail

typedef void *IRNETHANDLE;

#define MAX_PATH          260

struct DeviceInfo
{
	int id;      //Device Id
	char cName[MAX_PATH]; //the Device name
};

struct ComName
{
	char cComPort[MAX_PATH]; //the Device name
};


struct USBDeviceInfo
{
    uint16_t idVendor;
    uint16_t idProduct;
    uint8_t EndpointAddressOut;
    uint8_t EndpointAddressIn;

};

#define MAX_DEVICE_NUM	100

struct DeviceLst
{
	int iNumber;      //Device Count
    int usbCount;
    int comCount;
	DeviceInfo DevInfo[MAX_DEVICE_NUM];
    USBDeviceInfo USBInfo[MAX_DEVICE_NUM];
    ComName ComNameInfo[MAX_DEVICE_NUM];
};

typedef struct
{
	int emissivity;
	int airTemp;
	int reflectTemp;
	int humidity;
	int distance;
} envir_param; //Parameters are actual values * 10000

typedef enum
{
    TYPE_T1 = 1,
    TYPE_T2,
    TYPE_T3,
} FormulaType;

typedef enum
{
    TYPE_AT200_AT21F,
    TYPE_LT,
    TYPE_M3L,
    TYPE_M3256,
    TYPE_M3S,
    TYPE_M5,
    TYPE_M3,
    TYPE_GM_GC,
} DeviceType;
