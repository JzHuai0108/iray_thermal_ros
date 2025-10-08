#include <stdio.h>
#include <pthread.h>
#include <iostream>
#include <string>

//#include "main.h"
#include "include/InfEntity.h"
#include "include/USBSDK.h"


#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <thread> // for std::this_thread::sleep_for
#include <chrono> // for std::chrono::milliseconds
#include <std_msgs/String.h>

pthread_t thread_connect;

int iTempMeasType;
//int iTempUnit = 0;//0:Celsius; 1:Kelvin; 2:Fahrenheit;
int iCoreType = 0;//1: LT Temperature measurement type     2??MicroIII Temperature measurement type    3??MicroIII Imaging
IRNETHANDLE handle;
DeviceLst devList;

int iGetCurSel_init = 3;
int iGetCurSel = 3; // the infrared camera device id
int portIndex = 0; // the infrared camera port serial number
int vcount = 0;
ros::Publisher image_pub;
ros::Publisher image_pub1;
ros::Publisher tmp_image_pub;
ros::Publisher string_pub;
ros::Publisher string_pub_tem;
ros::Publisher string_pub_tem1;
ros::Publisher nuc_pub;

float cenTem = 0.0;
int index_cen; 
double thermal_tem = 0.12;
int color = 3;
bool isCloseNUC = false;

// 网格划分的行列数
int numRows = 16;
int numCols = 16;
// 存储网格中心坐标
std::vector<cv::Point> g_gridCenters;

int ReadCoreType(IRNETHANDLE p)
{

    unsigned char szCmd[0x08] =
    {
        0xAA, 0x04, 0x01, 0x70, 0x00, 0x1F, 0xEB, 0xAA
    };

    int errRet = WriteHandle(p, (char*)szCmd, 0x08);
    if (errRet != 0)
    {
        return errRet;
    }

    int t = 0;
TIP:
    sleep(MAX_WAIT_RESPONSE_TIME * 2);
    unsigned char recvBuf[MAX_LOCAL_BUF_LEN] = { 0 };
    int recvCnt = 0;
    errRet = ReadHandle(p, (char*)recvBuf, (char*)szCmd, &recvCnt, 0x08);
    if (errRet != 0)
    {
        t++;
        if (t < 4)
        {
            goto TIP;
        }
        return errRet;
    }
	printf("recvCnt:%d, recvBuf:%s", recvCnt, recvBuf);
}

int set_plate()
{
	unsigned char buf[] ={0xAA, 0x05, 0x01, 0x42, 0x02, 0x04, 0xF8, 0xEB, 0xAA};
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	char tempBuf[2048]; // 临时缓冲区
	unsigned char buf1[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf1, tempBuf, 9);
		// 输出读取到的数据
		for (int i = 0; i < 9; ++i) {
				std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf1[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "读取失败" << std::endl;
	}


	return result;
}

void getGridCord()
{
	// 计算每个网格单元的大小
	int rowStep = 512 / numRows;
	int colStep = 640 / numCols;

	// 计算每个网格的中心点坐标
	for (int i = 0; i < numRows; ++i) {
		for (int j = 0; j < numCols; ++j) {
			// 计算每个网格的起始坐标
			int startX = j * colStep;
			int startY = i * rowStep;

			// 计算每个网格的结束坐标
			int endX = (j + 1) * colStep;
			int endY = (i + 1) * rowStep;

			// 计算网格的中心点
			int centerX = (startX + endX) / 2;
			int centerY = (startY + endY) / 2;

			// 存储网格中心坐标
			g_gridCenters.push_back(cv::Point(centerX, centerY));

		}
	}
}

void getThermalTem(){
	double t = 0.1;
	unsigned char buf[] ={0xAA, 0x04, 0x01, 0x7C, 0x00, 0x2B, 0xEB, 0xAA};
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	// 等待 200 毫秒
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
	
	char tempBuf[2048]; // 临时缓冲区
	unsigned char buf1[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf1, tempBuf, 9);
		// 输出读取到的数据
		for (int i = 0; i < 9; ++i) {
				// std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf1[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "读取失败" << std::endl;
	}

	
    std::stringstream ss;
    ss << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<int>(buf1[4]);
    
    std::stringstream ss1;
    ss1 << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<int>(buf1[5]);

    // 组合两个 16 进制字符串
    std::string combinedHexStr = ss1.str() + ss.str();
  
    int x = std::stoi(combinedHexStr, nullptr, 16); // 将16进制字符串转换为10进制整数
    t = x / 100.0;
	
    
	std_msgs::String msg;
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2) << t;
	std::cout << "t:  " << stream.str() << std::endl;
	
    msg.data = stream.str();
    string_pub_tem1.publish(msg);
	return;
}

void calculateChecksumAndReplace(unsigned char buf[], int& size) {
    // 确保 buf 数组至少有 4 个字节，才能计算
    if (size < 4) {
        std::cout << "Error: buffer size is too small." << std::endl;
    }
	
	std::cout << "size:" << size << std::endl;
	for (int i = 0; i < size; ++i) {
		std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf[i])) << " ";
	}
	std::cout << std::endl;

    int sum = 0;

    // 计算 buf[] 从第一个字节到倒数第四个字节（包括倒数第四个字节）的和
    for (int i = 0; i < size - 3; ++i) {
        sum += static_cast<int>(buf[i]);
    }
	// std::cout << "sum:" << sum << std::endl;

    // 对和进行 256 取余
    int checksum = sum % 256;
	// std::cout << "256 res:" << checksum << std::endl;

    // 转换为 char 类型（如果需要用于字符输出）
    char checksumChar = static_cast<char>(checksum);

    // 输出调试信息
    // std::cout << "Checksum (decimal): " << checksum << std::endl;
    std::cout << "Checksum (hex): 0x" << std::hex << std::uppercase << checksum << std::endl;
    // std::cout << "Checksum (char): " << static_cast<unsigned char>(checksumChar) << std::endl;

    // 替换倒数第三位（校验位）
    buf[size - 3] = static_cast<unsigned char>(checksumChar);
	std::cout << "calculate buf:";
	for (int i = 0; i < size; ++i) {
		std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf[i])) << " ";
	}
	std::cout << std::endl;
    // 返回修改后的 buf 数组
}

void closeAutoNUC(){
	std::cout << "-----------set close AutoNUC-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x05, 0x01, 0x01, 0x01, 0x00, 0xB2, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);

	std::cout << "start write\n";
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	std::cout << "end write\n";
	char tempBuf[2048]; // 临时缓冲区
	unsigned char buf1[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));
	std::cout << "end ReadHandle\n";
	if(readresults == 0) {
		std::memcpy(buf1, tempBuf, 8);
		// 输出读取到的数据
		for (int i = 0; i < 8; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf1[i])) << " ";
		}
		std::cout << std::endl;
		isCloseNUC = true;
	} else {
		std::cout << "Error: read closeAutoNUC faild" << std::endl;
	}

	std::cout << "-----------set close AutoNUC-----------" << std::endl;
	return;
}

void setMaulNUC(){
	if(true == isCloseNUC)
	{
		std::cout << "\n-----------set Maul NUC-----------" << std::endl;
		unsigned char buf[] ={0xAA, 0x05, 0x01, 0x11, 0x02, 0x01, 0xC4, 0xEB, 0xAA};
		int size = sizeof(buf) / sizeof(buf[0]);
	
		calculateChecksumAndReplace(buf, size);
		
		int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
		
		// char tempBuf[2048]; // 临时缓冲区
		// unsigned char buf1[2048]; // 最终缓冲区
		// int len1 = 0;
		// int readresults = ReadHandle(handle,tempBuf, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

		// if(readresults == 0) {
		// 	std::memcpy(buf1, tempBuf, 9);
		// 	// 输出读取到的数据
		// 	for (int i = 0; i < 9; ++i) {
		// 		std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf1[i])) << " ";
		// 	}
		// 	std::cout << std::endl;
		// } else {
		// 	std::cout << "读取失败" << std::endl;
		// }
	}
	else
	{
		std::cout << "Auto NUC not close, can not maul NUC";
	}

	std::cout << "-----------set Maul NUC-----------" << std::endl;
}

void setDetail(){
	std::cout << "-----------set Detail-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x06, 0x02, 0x30, 0x01, 0x5A, 0x00, 0x15, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);
	
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf11[2048]; // 临时缓冲区
	unsigned char buf11[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf11, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf11, tempBuf11, 8);
		// 输出读取到的数据
		for (int i = 0; i < 8; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf11[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: set Detail faild" << std::endl;
	}

	std::cout << "-----------set Detail-----------" << std::endl;
	return;
}

void readDetail(){
	std::cout << "-----------read Detail-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x04, 0x02, 0x30, 0x00, 0xE0, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);
	
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf12[2048]; // 临时缓冲区
	unsigned char buf12[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf12, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf12, tempBuf12, 11);
		// 输出读取到的数据
		for (int i = 0; i < 11; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf12[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: read Detail faild" << std::endl;
	}
	std::cout << "-----------read Detail-----------" << std::endl;
	return;
}

void setDenoiseT1(){
	std::cout << "-----------set DenoiseT1-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x05, 0x02, 0x4B, 0x01, 0x78, 0x4D, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);
	
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf13[2048]; // 临时缓冲区
	unsigned char buf13[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf13, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf13, tempBuf13, 8);
		// 输出读取到的数据
		for (int i = 0; i < 8; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf13[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: set DenoiseT1 faild" << std::endl;
	}
	std::cout << "-----------set DenoiseT1-----------" << std::endl;
	return;
}

void readDenoiseT1(){
	std::cout << "-----------read DenoiseT1-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x04, 0x02, 0x4B, 0x00, 0xFB, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);
	
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf14[2048]; // 临时缓冲区
	unsigned char buf14[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf14, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf14, tempBuf14, 8);
		// 输出读取到的数据
		for (int i = 0; i < 8; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf14[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: read DenoiseT1 faild" << std::endl;
	}
	std::cout << "-----------read DenoiseT1-----------" << std::endl;
	return;
}

void setDenoiseT2(){
	std::cout << "-----------set DenoiseT2-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x05, 0x02, 0x4C, 0x01, 0x78, 0x62, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);
	
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf15[2048]; // 临时缓冲区
	unsigned char buf15[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf15, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf15, tempBuf15, 8);
		// 输出读取到的数据
		for (int i = 0; i < 8; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf15[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: set DenoiseT2 faild" << std::endl;
	}
	std::cout << "-----------set DenoiseT2-----------" << std::endl;
	return;
}

void readDenoiseT2(){
	std::cout << "-----------read DenoiseT2-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x04, 0x02, 0x4C, 0x00, 0xFC, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);

	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf16[2048]; // 临时缓冲区
	unsigned char buf16[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf16, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf16, tempBuf16, 8);
		// 输出读取到的数据
		for (int i = 0; i < 8; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf16[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: read DenoiseT2 faild" << std::endl;
	}
	std::cout << "-----------read DenoiseT2-----------" << std::endl;
	return;
}

void readFPATmp(){
	std::cout << "-----------read FPA Tmp-----------" << std::endl;
	unsigned char buf[] ={0xAA, 0x04, 0x01, 0xC3, 0x00, 0x72, 0xEB, 0xAA};
	int size = sizeof(buf) / sizeof(buf[0]);
	
	calculateChecksumAndReplace(buf, size);
	int result = WriteHandle(handle,reinterpret_cast<char*>(buf),sizeof(buf));
	
	char tempBuf17[2048]; // 临时缓冲区
	unsigned char buf17[2048]; // 最终缓冲区
	int len1 = 0;
	int readresults = ReadHandle(handle,tempBuf17, reinterpret_cast<char*>(buf),  &len1, sizeof(buf));

	if(readresults == 0) {
		std::memcpy(buf17, tempBuf17, 9);
		// 输出读取到的数据
		for (int i = 0; i < 9; ++i) {
			std::cout << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(buf17[i])) << " ";
		}
		std::cout << std::endl;
	} else {
		std::cout << "Error: read FPA Tmp faild" << std::endl;
	}
	std::cout << "-----------read FPA Tmp-----------" << std::endl;
	return;
}

void VideoCallBackfun(unsigned char *pBuffer, int width, int height, void* pContext)
{
	vcount++;
	if (vcount % 25 == 0){
		getThermalTem();
	}
	
	cv::Mat yuv422_mat(height, width, CV_8UC2, pBuffer);
	cv::Mat bgr_image;
	cv::Mat bgr_image_show;
	cv::cvtColor(yuv422_mat, bgr_image, cv::COLOR_YUV2BGR_UYVY); // "bgr8"

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg();
	msg->header.stamp = ros::Time::now();
	image_pub.publish(msg);

	bgr_image.copyTo(bgr_image_show);

	cv::line(bgr_image_show, cv::Point(width/2 - 10, height/2), cv::Point(width/2 + 10, height/2), cv::Scalar(255, 255, 255), 2);
	cv::line(bgr_image_show, cv::Point(width/2, height/2 - 10), cv::Point(width/2, height/2 + 10), cv::Scalar(255, 255, 255), 2);
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2) << cenTem;
	std::string text = stream.str();
	
	if (vcount % 10 == 0){
		std_msgs::String msg_cenTem;
		msg_cenTem.data = stream.str();
	    	string_pub_tem.publish(msg_cenTem);
	}
	
    	
	cv::putText(bgr_image_show, text, cv::Point(width/2 + 20, height/2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
	sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image_show).toImageMsg();
	msg1->header.stamp = ros::Time::now();
	msg1->header.frame_id = text;
	image_pub1.publish(msg1);
	
	if(vcount==5000){
		vcount = 0;
	}
	

}


int binaryToDecimal(unsigned char byte1, unsigned char byte2) {
    return (byte2 << 8) | byte1;
}


void TempCallBackfun(unsigned char *pBuffer, int width, int height, void* pContext)
{
	if(vcount%10==0){
		float decimalValue = binaryToDecimal(pBuffer[index_cen], pBuffer[index_cen + 1]) /10 -273.2;
		cenTem =  round(decimalValue * 100)/100 ;


		// 初始化最大值和最小值
		int maxTempBinary = std::numeric_limits<int>::min();
		int minTempBinary = std::numeric_limits<int>::max();
		// 遍历所有像素，查找最大和最小的二进制温度值
		for (int i = 0; i < width * height * 2; i += 2) {
			int tempBinary = binaryToDecimal(pBuffer[i], pBuffer[i + 1]);

			if (tempBinary > maxTempBinary) {
				maxTempBinary = tempBinary;
			}
			if (tempBinary < minTempBinary) {
				minTempBinary = tempBinary;
			}
		}

		// 将二进制温度值转换为实际温度值
		float maxTemp = round((maxTempBinary / 10.0f - 273.2f) * 100) / 100;
		float minTemp = round((minTempBinary / 10.0f - 273.2f) * 100) / 100;

		int tmpIdx = 0;
		int tmpValue = 0;
		
		// 模拟获取的网格中心像素值，长度为 256
		std::vector<int> vCenterPixelValue(256);

		for(int gridIdx = 0; gridIdx < g_gridCenters.size(); gridIdx++)
		{
			tmpIdx = g_gridCenters[gridIdx].y * 640 * 2 + g_gridCenters[gridIdx].x * 2;
			tmpValue = binaryToDecimal(pBuffer[tmpIdx], pBuffer[tmpIdx + 1]) /10 -273.2;
			if(tmpValue <= 0)
			{
				vCenterPixelValue[gridIdx] = 0;
			}
			else
			{
				vCenterPixelValue[gridIdx] = tmpValue;
			}
		}
		
	    // 创建一个 16x16 的 cv::Mat，用 CV_32S 类型表示整数数据
		cv::Mat reconstructedImage(16, 16, CV_32S);

		// 将 vector 中的值按行优先的顺序填充到 cv::Mat 中
		for (int i = 0; i < 16; ++i) {
			for (int j = 0; j < 16; ++j) {
				reconstructedImage.at<int>(i, j) = vCenterPixelValue[i * 16 + j];
			}
		}

		// ------------------for test 640*512--------------------
		// int imageWidth = 640;
		// int imageHeight = 512;

		// // 创建空图像，初始值为0
		// cv::Mat reconstructedImage(imageHeight, imageWidth, CV_8UC1, cv::Scalar(0));

		// // 网格每个块的宽度和高度
		// int rowStep = imageHeight / numRows;
		// int colStep = imageWidth / numCols;
		// std::cout << "----------------\n";
		// // 遍历每个网格块的中心
		// for (int gridIdx = 0; gridIdx < g_gridCenters.size(); gridIdx++) {
		// 	// 获取网格中心的坐标
		// 	int centerX = g_gridCenters[gridIdx].x;
		// 	int centerY = g_gridCenters[gridIdx].y;

		// 	// 计算该块的边界
		// 	int startX = std::max(centerX - colStep / 2, 0);
		// 	int startY = std::max(centerY - rowStep / 2, 0);
		// 	int endX = std::min(centerX + colStep / 2, imageWidth - 1);
		// 	int endY = std::min(centerY + rowStep / 2, imageHeight - 1);

		// 	// 计算tmpValue（假设pBuffer已经加载数据）
		// 	int tmpIdx = centerY * imageWidth * 2 + centerX * 2;
		// 	int tmpValue = binaryToDecimal(pBuffer[tmpIdx], pBuffer[tmpIdx + 1]) / 10 - 273.2;
		// 	std::cout << tmpValue << " ";
		// 	// 将该块内的所有像素值设置为tmpValue
		// 	for (int y = startY; y <= endY; y++) {
		// 		for (int x = startX; x <= endX; x++) {
		// 			reconstructedImage.at<unsigned char>(y, x) = static_cast<unsigned char>(tmpValue);
		// 		}
		// 	}
		// }
		// ------------------for test--------------------



		sensor_msgs::ImagePtr tmp_msg = cv_bridge::CvImage(std_msgs::Header(), "32SC1", reconstructedImage).toImageMsg();
		tmp_msg->header.stamp = ros::Time::now();
		// tmp_msg->header.frame_id = text;
		tmp_image_pub.publish(tmp_msg);

		// 使用 ostringstream 格式化温度值
		std::ostringstream stream1;
		stream1 << std::fixed << std::setprecision(2) << maxTemp;

		std::ostringstream stream2;
		stream2 << std::fixed << std::setprecision(2) << minTemp;

		// 将 ostringstream 转换为字符串
		std::string maxTempStr = stream1.str();
		std::string minTempStr = stream2.str();
		std::string pub_temp = maxTempStr + "," + minTempStr;

		std_msgs::String msg;
		msg.data = pub_temp;  
		string_pub.publish(msg);
	}
	

}

int connectdevice()
{
	if((devList.iNumber > 0)&&((devList.comCount > 0) || (devList.usbCount > 0)))
	{
		int connectType = 1;
		// int iGetCurSel = 1;
		// int portIndex = 0;
		int usbIndex = 0;
		//if(devList.comCount > 0)
		//{
		//	connectType = 0;
		//}
		//else
		//{
		//	connectType = 1;
		//}

		SetVideoCallBack(handle, VideoCallBackfun, NULL);
		SetTempCallBack(handle, TempCallBackfun, NULL);
		int ret = OpenDevice(handle, connectType, iGetCurSel, portIndex, usbIndex);
		printf("open result: %d\n", ret);
		return ret;


	}
	return -1;
}

// 新线程函数，用于每隔 2 分钟调用一次
void setMaulNUC_periodically() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(60));
		std_msgs::String msg_nuc;
		std::string nuc = "nuc";
		msg_nuc.data = nuc;
		nuc_pub.publish(msg_nuc);
		std::this_thread::sleep_for(std::chrono::seconds(2));
		std::cout << "nuc once" << std::endl;
		setMaulNUC();
	}
}

int main(int argc, char *argv[])
{
	if (argc >= 2) {
		iGetCurSel_init = std::atoi(argv[1]);
		if (argc >=3 ) {
			portIndex = std::atoi(argv[2]);
		}
	} else {
		printf("Use: sudo env LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu \\\n"
			   "%s [deviceid=1] [portid=0]\n", argv[0]);
		printf("port id does not seem to matter much, and 0 is valid most times.\n");
	}
	printf("Device id %d, port index %d\n", iGetCurSel, portIndex);

	ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_pub = nh.advertise<sensor_msgs::Image>("/iray/thermal_img", 1000);
	image_pub1 = nh.advertise<sensor_msgs::Image>("/iray/thermal_img_show", 1000);
	tmp_image_pub = nh.advertise<sensor_msgs::Image>("/iray/tmp_img", 1000);
	string_pub = nh.advertise<std_msgs::String>("/maxmin_tem", 10);
	string_pub_tem1 = nh.advertise<std_msgs::String>("/thermal_tem1", 10);
	string_pub_tem = nh.advertise<std_msgs::String>("/thermal_tem", 10);
	nuc_pub = nh.advertise<std_msgs::String>("/iray/nuc", 10);
	index_cen = 640*2*255+640;

	getGridCord();

	printf("start!\n");

	int rc = 0;

	handle = sdk_create();

	printf("create!%x\n", handle);
	sdk_loginDevice(handle);
	//DeviceLst devList;

	printf("login!\n");
	int iRes = SearchDevice(handle, devList);
	if(devList.iNumber > 0)
	{
		for (int i = 0; i < devList.iNumber; i++)
		{
			printf("DevInfo: %d %s\n", i, devList.DevInfo[i].cName);
			std::string device = devList.DevInfo[i].cName;
			char last = device.back();
			int ld = last - '0';
			printf("%d \n", ld);
			if(ld == iGetCurSel_init){
				iGetCurSel = i;
				printf("select %d \n", i);
			}
		}
	}

	if(devList.comCount > 0)
	{
		for (int i = 0; i < devList.comCount; i++)
		{
			printf("ComNameInfo:  %s\n", devList.ComNameInfo[i].cComPort);
		}
	}
	if(devList.usbCount > 0)
	{
		for (int i = 0; i < devList.usbCount; i++)
		{
			printf("USBInfo:  %x\n", devList.USBInfo[i].idVendor);
		}
	}
	rc = connectdevice();

    int iType = CoreType(handle); // 2 for micro iii temperature measuring type
	int color_plate = -47;
	int getres = sdk_get_color_plate(handle, iType, &color_plate); // this function does not return a meaningful value at color_plate.
	int new_color_plate = 0; // 0 for white hot, 4 for iron
	int setres = sdk_set_color_plate(handle, iType, new_color_plate); // this function returns 0 meaning 
	printf("coretype %d sdk_get_color res %d, color plate %d, set color plate res %d\n", 
			iType, getres, color_plate, setres);

	return -1;


	sleep(2);
	set_plate();
	// sdk_set_color_plate(handle, iCoreType, 15);
	// test read core type
	// ReadCoreType(handle);

	
	// set detail enhance
	setDetail();
	// read detail params
	readDetail();
	// setDenoiseT1
	setDenoiseT1();

	// readDenoiseT1
	readDenoiseT1();
	
	// setDenoiseT2
	setDenoiseT2();
	// readDenoiseT2
	readDenoiseT2();

	readFPATmp();

	// close auto NUC
	closeAutoNUC();
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	// setMaulNUC();
	
	std::cout << "end setting" << std::endl;

	// 启动新线程，每 2 分钟执行一次
    std::thread periodic_thread(setMaulNUC_periodically);

	int x;
	printf("输入1进行handle调试,输入其它退出程序");

	while(1){
		char input;
		input = getchar();
		if (input=='1'){
				unsigned char szCmd[0x08];
				for(int i=0; i<10; i++) {
					cout << "输入cmd: ";
					cin >> szCmd[i];
				}

				int errRet = WriteHandle(handle, (char*)szCmd, 0x08);
				if (errRet != 0)
				{
					return errRet;
				}

				int t = 0;
			TIP:
				sleep(MAX_WAIT_RESPONSE_TIME * 2);
				unsigned char recvBuf[MAX_LOCAL_BUF_LEN] = { 0 };
				int recvCnt = 0;
				errRet = ReadHandle(handle, (char*)recvBuf, (char*)szCmd, &recvCnt, 0x08);
				if (errRet != 0)
				{
					t++;
					if (t < 4)
					{
						goto TIP;
					}
					return errRet;
				}
				printf("recvCnt:%d, recvBuf:%s", recvCnt, recvBuf);
				printf("输入1继续");
		}else if (input=='5')
		{
			int colortype = (color++%20);
			sdk_set_color_plate(handle, iCoreType, colortype);
			printf("switch color!\n");
		}
		

		sleep(0.5);

	}

	return 0;

}
