#pragma once
#include "BaseEncoderCtrl.h"
#include "InfEntity.h"
#ifdef MY_LINUX
#include <libusb-1.0/libusb.h>
#endif

#define FRAME_LT_START          (0xAA)
#define FRAME_LT_BACK           (0x55)
#define FRAME_TYPE              (0x00)
#define FRAME_TYPE01              (0x01)
#define FRAME_END_EB         (0xEB)
#define FRAME_END_AA         (0xAA)

#define MAX_WAIT_RESPONSE_TIME   100
#define MAX_LOCAL_BUF_LEN		512
static int mPaletteIndex = 0;//伪彩序号
static int m_coreType = 0;//1: LT Temperature measurement type     2：MicroIII Temperature measurement type    3：MicroIII Imaging     4:ELF3


static unsigned char palettedata[40 * 1024] = {0};//伪彩文件
static void* outContext = NULL;
typedef enum IR_OPERATE_e  //操作子枚举
{
	IR_OPERATE_READ = 0x00,
	IR_OPERATE_WRITE = 0x01,
	IR_OPERATE_DO = 0x02,
	IR_OPERATE_READ_CK = 0x03,
	IR_OPERATE_WRITE_CK = 0x04,
} IR_OPERATE, *P_IR_OPERATE;

#define LT_OPERATE_COMMAND0             0x07
#define CMD_LT_TEMP_FRAME_MAX_MES       0x27 //整帧测温最大温度/坐标(read only)
#define CMD_LT_TEMP_FRAME_MIN_MES       0x29 //整帧测温最小温度/坐标(read only)
#define CMD_LT_TEMP_FRAME_CEN_MES       0x2c //整帧测温中心温度/坐标(read only)
#define CMD_LT_TEMP_FRAME_AVR           0x2a //整帧测温均值温度     (read only)
#define CMD_LT_TEMP_USER_REFLECT        0x0f //反射温度
#define CMD_LT_TEMP_USER_AIRTEMP        0x10 //大气环境温度
#define CMD_LT_TEMP_USER_HUMIDITY       0x11 //环境湿度
#define CMD_LT_TEMP_USER_EMISS          0x12 //发射率
#define CMD_LT_TEMP_USER_DISTANCE       0x13 //距离
#define CMD_LT_CALCULATE_ENV_PARAMS     0x18 //环境变量修正重新计算
#define CMD_LT_WTR_LOW_THRESHOLD        0x1d
#define CMD_LT_WTR_HIGH_THRESHOLD       0x1e
#define CMD_LT_TEMP_SWITCH              0x71 //温度成像T值切换使能
#define CMD_LT_TEMP_WTR                 0xf0
#define CMD_LT_TEMP_DISPLAY_STYLE       0x02 //温度显示样式：0x00-摄氏度 C，0x01-开尔文（273.15K），0x02-华氏（F）

#define BULK_ENDPOINT_OUT               0x05
#define BULK_ENDPOINT_IN                0X85

typedef enum IR_FEATURE_e	//公有指令集定义
{
	//  命令字                                      值   操作  功能说明    参数字节数                 说明（默认返回 OK）
	IR_FEATURE_USER_INFO_SN = 0,		//0   WR,机芯的SN码 长度目前使用6位    下载和上传时 通过命令的长度判断使用长度
	IR_FEATURE_USER_INFO_PN,        //1   WR,机芯的PN码 长度目前使用14位   下载和上传时 通过命令的长度判断使用长度  举例：LB6230CTT20001
	IR_FEATURE_USER_INFO_WIDTH,        //2   R, 探测器FPA的宽度   0            只读参数，返回参数2字节
	IR_FEATURE_USER_INFO_HEIGHT,        //3   R, 探测器FPA的搞度   0            只读参数，返回参数2字节
	IR_FEATURE_USER_TMPR_FPA,        //4   R, FPA温度           0            只读参数，返回参数2字节  温度=返回数据/100
	IR_FEATURE_USER_TMPR_ENV,        //5   R, 机芯温度          0            只读参数，返回参数2字节  返回数据=温度*100
	IR_FEATURE_USER_TEMPSENSOR_TYPE,			//6   R， 返回参数一个 ：0x88表示内部温传，0x80表示外部温传
	IR_FEATURE_NUC_STATUS_EN = 14,//        			    //14   W, bit0 图像稳定开关 0 dis 1 en
	IR_FEATURE_USER_RESERVED_7_TO_15,		//7-15 预留待定

	IR_FEATURE_USER_SETT_UPLOAD = 16,        //16   预留待定
	IR_FEATURE_USER_SETT_SAVE,        //17  W, 参数保存功能      0            将用户配置参数保存到flash，下位机将当前配置保存
	IR_FEATURE_USER_SETT_RESTORE_FACTORY,       //18  DO 恢复出厂设置      0            将出厂配置参数恢复到当前配置
	IR_FEATURE_USER_SETT_RESTART,               //19  DO 机芯重启          0             机芯重启
	IR_FEATURE_USER_SETT_BAUD,                  //20  DO 修改通讯波特率    1            参数 0x01:自动识别(可空缺 但是要返回识别失败) 0x02:9600 0x04:19200 0x08 :38400 0x10:115200 0x20:921600

	IR_FEATURE_USER_NUC_AUTOSHUT,        //21 WR 设置自动/手动快门 1              参数1字节，0：手动打 1：自动打
	IR_FEATURE_USER_NUC_SHUT_TYPE,        //22 WR 手动快门类型      1              参数1字节，0/1：快门校正 2：背景校正
	IR_FEATURE_USER_NUC_TIME_INTERVAL,        //23 WR 自动快门时间间隔  1
	IR_FEATURE_USER_NUC_TEMP_CHANGE,        //24 WR 自动快门温度间隔  1

	IR_FEATURE_USER_TEC_CHANGE_EN = 25,        //25 WR TEC自动切换开关    1             参数1字节  0，关 1 开
	IR_FEATURE_DRC_CLASS = 25,  		   					    //25   DO：  参数1个，DRC档数选择，总共16档，包含dde和drc中部分参数 
																//如果为0，则使用用户可调整的那组值，否则使用flash中的1-16档预设定值。

	IR_FEATURE_USER_INFO_VERSION,        //26 R  读取当前设备版本 
	IR_FEATURE_USER_DISCONNECT,        //27 DO  断开串口连接  无返回数据 
	R_FEATURE_USER_RESERVED_28_TO_32,          //29-41 预留待定	 //IR_FEATURE_USER_TEST_IMAGE         ,      
																										   //28 DO  测试图像开关      1             参数1字节  0，关 1 开
	IR_FEATURE_DRC_DDE_DETAIL_GAIN = 30,        				//30  W,参数1个，设置细节增益系数，5位整数，3位小数
	IR_FEATURE_DRC_AGC_HIGHTHROWPERCENT_SET = 33,			//33  W,参数1，设置线性直方图的高抛比例lowThrowPercent,单位是1/1024.
	IR_FEATURE_DRC_AGC_HPLATEAUVALUE = 36,					//36  W,PE的高平台值
	IR_FEATURE_DRC_AGC_LPLATEAUVALUE = 37,					//37  W,PE的低平台值
	IR_FEATURE_DRC_AGC_ITTMEAN = 38,							//38  W,  1个参数 ITTMEAN
	IR_FEATURE_DRC_AGC_MAXGAIN_SET = 40,						//40  W,设置AGC中查找表的最大增益
	IR_FEATURE_USER_RESERVED_38_TO_41,          //38-41 预留待定

	IR_FEATURE_USER_VIDEO_ZOOM = 42,        //42  WR 电子变倍          1             0:不变倍 1：2倍 2：4倍 3：8倍
	IR_FEATURE_USER_VIDEO_CROSS_EN,        //43  WR 十字光标开关      1             0：打盲元关 1 打盲元开 0x02：用户光标关， 0x03：用户光标开
	IR_FEATURE_USER_VIDEO_CROSS_MOVE,        //44  DO 十字光标动作      1             盲元标定 0：中心 1：向上 2：向下 3：向左 4：向右 最高位BIT7 0 最小步幅 1 最大步幅；  用户光标 5：中心 6：向上 7：向下 8：向左 9：向右 最高位BIT7 0 最小步幅 1 最大步幅
	IR_FEATURE_USER_VIDEO_COLORMAP,        //45  WR 调色板设置        1             参数符合COLORMAP_TYPE_e
	IR_FEATURE_USER_DIGITAL_SOURCE,        //46 WR 数字视频源        1               参数符合DIGITAL_SOURCE_e
	IR_FEATURE_USER_DIGITAL_TYPE,        //47 WR 数字视频输出接口  1               参数符合DIGITAL_TYPE_e 
	IR_FEATURE_USER_VIDEO_FLIP,		//48 WR 图像翻转          1              8bit参数 其中 bit0 = 1不翻转，bit1=1左右翻转，bit2=1上下，bit3=1左上右下
	IR_FEATURE_USER_BF_EN,        //49 WR 滤波开关            1            双边滤波开关（上位机显示滤波开关功能） 0 关 1开
	IR_FEATURE_USER_AGC_FROZEN,        //50  DO 参数一：00 冻结 01 解除冻结 
	IR_FEATURE_USER_OSD_VIDEO_EN,        //51 WR 模拟视频开关 just for LT
	IR_FEATURE_USER_DIGITAL_TYPE_LT,        //52 WR LT视频
	IR_FEATURE_USER_RESERVED_52_TO_57,        //53-57 预留待定

	IR_FEATURE_USER_AGC_MODE = 58,   	    //58 WR：AGC模式，        1               参数符合AGC_ALGORITHM_e（0手动，1自动0（PE），2自动1(LE)）
	IR_FEATURE_USER_AGC_CONTRAST,   	    //59 WR,设置对比度        1               范围0-255
	IR_FEATURE_USER_AGC_BRIGHT,	   	//60  WR, 设置亮度，       2               范围0-511
	IR_FEATURE_USER_AGC_GAMMA = 61,		//61  WR, GAMMA校正        1               设置gamma校正指数，真正的指数乘以10，即单位0.1 软件界面显示 1.1 下发 11
	IR_FEATURE_USER_AGC_ANALOG_ON_OFF = 61,		//61 模拟视频开关 
	IR_FEATURE_USER_DDE_EN,		//62  WR  DDE状态          1               0：关 1：开
	IR_FEATURE_USER_DDE_GRADE,        //63  WR  DDE档位          1               目前范围0-7 可以扩展
	IR_FEATURE_USER_AGC_CONTRAST_STEP,        //64 对比读步长值
	IR_FEATURE_USER_AGC_BRIGHTNESS_STEP,        //65 亮度步长值
	IR_FEATURE_OSD_CURSOR_MOVE = 68,
	IR_FEATURE_USER_RESERVED_66_TO_80,        //66-80 预留待定

	IR_FEATURE_USER_BLIND_SCAN = 81,		//81  DO   盲元扫描        0              
	IR_FEATURE_USER_BLIND_OP,        //82  DO   盲元下载操作    1               0：设置当前盲元 1：撤销上次设置 2： 保存盲元表到FLASH

	IR_FEATURE_USER_K_IMAGE,       //83  DO  获取当前图像数据 1               当操作为DO时 获取当前图像为 0：低温 1：高温数据 计算K值
	IR_FEATURE_USER_K_CAL,       //84  DO  自动计算K值
	IR_FEATURE_USER_K_OP,       //85  DO  用户K的相关操作  1               0：保存当前K值 1：恢复出厂K值 （其他 上传/下载 K值 暂不使用 预留扩展）

	IR_FEATURE_USER_FIRM_UPLOAD,       //86  W/D  固件更新      W:128/D:2      固件更新大小(KB)次数=大小*8 具体实现 上位机发送 此指令DO 返回成功之后，开始使用该指令的W 操作 每次发送128B数据 共发送8*大小 次

	IR_FEATURE_USER_FIRM_UPLOADK,       //87  W/R/D  快速固件更新   W/R/D          固件更新大小(KB)次数=大小 具体实现 上位机发送 此指令的W操作 发送次数（一次1K） 返回成功之后，开始下载操作 每次发送1KB数据（只有固件数据） 共发送大小 次 
	//发送完成后，发送指令的Read操作读取下位机数据校验(4字节校验)，校验失败提示重新下载，成功保存，下发DO操作保存到flash
	IR_FEATURE_USER_RESERVED_88_TO_99,       // 88-99 预留待定

	IR_FEATURE_USER_KEY = 100,       // 100 DO  虚拟按键        1               最高位bit7 作为长按标记 长按1 单按 0  键位对应 bit0:M键 bit1: +键  bit2:-键 bit3:C键
	IR_FEATURE_USER_BLACKFACE_SWITCH = 124,       // 100 DO  虚拟按键        1               最高位bit7 作为长按标记 长按1 单按 0  键位对应 bit0:M键 bit1: +键  bit2:-键 bit3:C键
	IR_FEATURE_USER_BLACKFACE_TEMP = 125,       // 100 DO  虚拟按键        1               最高位bit7 作为长按标记 长按1 单按 0  键位对应 bit0:M键 bit1: +键  bit2:-键 bit3:C键
	IR_FEATURE_USER_BLACKFACE_LOCATION = 126,       // 100 DO  虚拟按键        1               最高位bit7 作为长按标记 长按1 单按 0  键位对应 bit0:M键 bit1: +键  bit2:-键 bit3:C键
	IR_FEATURE_USER_RESERVED_101_TO_159, //101-159 基本功能增加预留预留待定
	IR_FEATURE_NS_FLASH_COLORMAP = 140,        				//140 10组色板
	IR_FEATURE_NS_FLASH_LOGO = 141,//LOGO下载
	IR_FEATURE_TEMPER_ALARM_EN = 160,      // 160 wr  高温报警使能     1               0关 1开
	IR_FEATURE_NS_AUTO_CALC_GG,							//161	获取锅盖 W，0获取并计算，1保存 2清除
	IR_FEATURE_MOTOR_MOVE = 162,     //162  DO  镜头电机控制      4              参数1 1字节 值0：停止 1 正转 2 反转 ；参数2 1个字节 设置PWM占空比 参数3 2个字节 设置步长 即电机转动毫秒数
	IR_FEATURE_NS_CALC_GG_PARA = 162,//162 写 ： 参数一  1.设置_系数A  2.设置 _系数B  3.设置_Range 4.设置温差上限 LimitMaxT 
								//读： 参数一 1. 读取_A 2.读取_B  3. 读取_renge 4.读取温度上限MaxT

	IR_FEATURE_RTD323_FPA_AVERAGE = 196, 					 //196(原176)	 R,读取阵列平均值（count）	 2个字节
	IR_FEATURE_RTD323_TEC_TEMP = 206,				 //206(原222)		 WR,设置TEC温度

	IR_FEATURE_RESERVED_163_TO_255,     //163 客户定制功能 预留待定

} IR_FEATURE_USER, *P_IR_FEATURE_USER;

#ifdef MY_WIN32
class DwEncoderCtrl : public BaseEncoderCtrl, public ISampleGrabberCB
#elif defined MY_LINUX
class DwEncoderCtrl : public BaseEncoderCtrl
#endif
{
public:
	DwEncoderCtrl();
	~DwEncoderCtrl();

	virtual bool Init() override;
	virtual void Deinit() override;
	virtual unsigned int Login(HWND hWnd) override;
	virtual int SearchDev(DeviceLst &p) override;
    virtual int OpenDev(int type, int iGetCurSel, int portIndx, int usbIndx) override;
	virtual void CloseDev() override;
	virtual void Logout() override;
	virtual int ReadCoreType() override;
	
	virtual int ReadTempMeasureType() override;
	virtual int shutter_correction(int iCoreType, int type) override;
	virtual int set_color_plate(int iType, int color_plate) override;
	virtual int get_color_plate(int iType, int* color_plate) override;
	virtual int get_SN_PN(char *strSN, int *iLenSN, char* strPN, int *iLenPN) override;

	int __stdcall StartDeviceVideoCallBack(VideoCallBack pVideoCallBack, void *pContext) override;
	int __stdcall StartTempCallBack(TempCallBack pTempCallBack, void *pContext) override;

    virtual int ReadCMDHandle(char* buf, char* cmd, int* pLen, int cmdsize) override;
    virtual int WriteCMDHandle(char* buf, int len) override;

	int OpenCom(int port, DWORD baud, int parity);
	int PingPong();
#ifdef MY_WIN32
	void EnumCom();
	int GetComList_Reg();
	virtual int USBVersion(/*char* version*/) override;
	virtual bool ReadCommunicationType() override;
	ULONG STDMETHODCALLTYPE AddRef();
	ULONG STDMETHODCALLTYPE Release();
	HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
	HRESULT STDMETHODCALLTYPE SampleCB(double Time, IMediaSample *pSample);
	HRESULT STDMETHODCALLTYPE BufferCB(double Time, BYTE *pBuffer, long BufferLen);
#endif // MY_WIN32


	int get_FPA_temp(float *fTemp) override;
	int get_camera_temp(float *fTemp) override;
	int get_width(int *iValue) override;
	int get_height(int *iValue) override;
	int set_wtr_status(int iStatus) override;
	int get_wtr_status(int* iStatus) override;
	int set_wtr_low_threshold(int iThreshold) override;
	int get_wtr_low_threshold(int* iThreshold) override;
	int set_wtr_high_threshold(int iThreshold) override;
	int get_wtr_high_threshold(int* iThreshold) override;
	int set_reflect(signed int i32value) override;
	int get_reflect(signed int* p32value) override;
	int set_airTemp(signed int i32value) override;
	int get_airTemp(signed int* p32value) override;
	int set_humidity(signed int i32value) override;
	int get_humidity(signed int* p32value) override;
	int set_emiss(signed int i32value) override;
	int get_emiss(signed int* p32value) override;
	int set_distance(signed int i32value) override;
	int get_distance(signed int* p32value) override;
	int set_envir_param(envir_param envir_data) override;
	int get_envir_param(envir_param* envir_data) override;
	int envir_effect() override;
	int read_temp_unit(unsigned char* ucUnit) override;
	int set_temp_unit(unsigned char ucUnit) override;
	int get_temp_coefficient(int gain, short* param1, short* param2) override;
	int edge_detect(unsigned char* imageSrc, unsigned char* imageDst, int width, int height, int level) override;
	int edge_enhace(unsigned char* imageSrc, unsigned char* imageDst, int width, int height, int level) override;
	int get_video_interface() override;
    int get_formulatype(int gain, int* k, int* b) override;

public:
    VideoCallBack vcb = NULL;
    TempCallBack tcb = NULL;
	DeviceLst devList;

private:
    int iIfGetVideoSize = 0;
    int Width = 640;
    int Height = 512;
#ifdef MY_WIN32
	CaptureVideo g_CaptureVideo;
	bool bUSBNew;
	int DataReadLen;
	int comport;
	DWORD oldbaud;
	DCB dcb;
	HANDLE m_hComm = (HANDLE)INVALID_HANDLE_VALUE;

    unsigned char camera_data_out[2560 * 1024 * 2];
    unsigned char yuv_data_out[1280 * 1024 * 2];
    unsigned char temp_data_temp_out[1280 * 1024 * 2];

	int iIfTempCore = 0; //是否是测温型机芯
	int m = 0;

	BOOL m_bGetPicture;  // is get a picture
	long m_lWidth;       //存储图片的宽度
	long m_lHeight;		 //存储图片的长度
	int  m_iBitCount;    //the number of bits per pixel (bpp)
	TCHAR m_chTempPath[MAX_PATH];
	TCHAR m_chSwapStr[MAX_PATH];
	TCHAR m_chDirName[MAX_PATH];
	bool bVideoTempFlag = false;
	vector<char*> name;
#elif defined MY_LINUX
	v4l2_dev_t *vd;
    serialport *sport;
	int fd;
	pthread_t thread_read;
    libusb_device_handle *dev_handle;
    libusb_context *ctx = nullptr;
#endif
	
	bool bTempMeasureType;//测温类型  true：人体测温  false： 工业测温
	bool m_CDS3;
    int m_deviceType = TYPE_M3;
    bool m_vidSupport = true;;
	int m_connectType;
};

