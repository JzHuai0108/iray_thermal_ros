#pragma once
#include "USBSDK.h"
#include "InfEntity.h"

class BaseEncoderCtrl
{
public:
	BaseEncoderCtrl();
	~BaseEncoderCtrl();

	virtual bool Init() { return m_inited = true; }
	virtual void Deinit() {}

	virtual unsigned int Login(HWND hWnd) { return 1; }
	virtual int SearchDev(DeviceLst &p){ return 1; }
    virtual int OpenDev(int type, int iGetCurSel, int portIndx, int usbIndx) { return 1; }
	virtual void CloseDev() { return; }
	virtual void Logout() { return; }
#ifdef MY_WIN32
	virtual int USBVersion(/*char* version*/) { return 1; }
	virtual bool ReadCommunicationType() { return 1; }
#endif // MY_WIN32
	virtual int ReadCoreType() { return 1; }
	virtual int ReadTempMeasureType() { return 1; }
	virtual int shutter_correction(int iCoreType, int type) { return 1; }
	virtual int set_color_plate(int iType, int color_plate) { return 1; }
	virtual int get_color_plate(int iType, int* color_plate) { return 1; }
	virtual int get_SN_PN(char *strSN, int *iLenSN, char* strPN, int *iLenPN) { return 1; }

	virtual int __stdcall StartDeviceVideoCallBack(VideoCallBack pVideoCallBack, void *pContext) { return 1; }
	virtual int __stdcall StartTempCallBack(TempCallBack pTempCallBack, void *pContext) { return 1; }

    virtual int ReadCMDHandle(char* buf, char* cmd, int* pLen, int cmdsize) { return 1; }
    virtual int WriteCMDHandle(char* buf, int len) { return 1; }

	virtual int get_FPA_temp(float *fTemp) { return 1; }
	virtual int get_camera_temp(float *fTemp) { return 1; }
	virtual int get_width(int *iValue) { return 1; }
	virtual int get_height(int *iValue) { return 1; }
	virtual int set_wtr_status(int iStatus) { return 1; }
	virtual int get_wtr_status(int* iStatus) { return 1; }
	virtual int set_wtr_low_threshold(int iThreshold) { return 1; }
	virtual int get_wtr_low_threshold(int* iThreshold) { return 1; }
	virtual int set_wtr_high_threshold(int iThreshold) { return 1; }
	virtual int get_wtr_high_threshold(int* iThreshold) { return 1; }
	virtual int set_reflect(signed int i32value) { return 1; }
	virtual int get_reflect(signed int* p32value) { return 1; }
	virtual int set_airTemp(signed int i32value) { return 1; }
	virtual int get_airTemp(signed int* p32value) { return 1; }
	virtual int set_humidity(signed int i32value) { return 1; }
	virtual int get_humidity(signed int* p32value) { return 1; }
	virtual int set_emiss(signed int i32value) { return 1; }
	virtual int get_emiss(signed int* p32value) { return 1; }
	virtual int set_distance(signed int i32value) { return 1; }
	virtual int get_distance(signed int* p32value) { return 1; }
	virtual int set_envir_param(envir_param envir_data) { return 1; }
	virtual int get_envir_param(envir_param* envir_data) { return 1; }
	virtual int envir_effect() { return 1; }
	virtual int read_temp_unit(unsigned char* ucUnit) { return 1; }
	virtual int set_temp_unit(unsigned char ucUnit) { return 1; }
	virtual int get_temp_coefficient(int gain, short* param1, short* param2) { return 1; }
	virtual int edge_detect(unsigned char* imageSrc, unsigned char* imageDst, int width, int height, int level) { return 1; }
	virtual int edge_enhace(unsigned char* imageSrc, unsigned char* imageDst, int width, int height, int level) { return 1; }
	virtual int get_video_interface() { return 1; }
    virtual int get_formulatype(int gain, int* k, int* b) { return 1; }

protected:
	volatile bool m_inited = false;
};

