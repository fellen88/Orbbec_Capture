// Orbbec_test.cpp : Defines the entry point for the console application.

// 预编译头文件
#include "stdafx.h"

// 标准库头文件  
#include <iostream>  
#include <string>  
#include <vector>   

// PCL头文件
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>

// OpenCV头文件  
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>  

// OpenNI头文件  
#include <OpenNI.h>   

typedef unsigned char uint8_t;

#define PI 3.141592689793

//1. 如果深度图与彩色没有做对齐 ，公式中的 FOV 应使用 depth FOV ;
//2. 如果深度图与彩色图已做对齐 ，公式中的 FOV 应使用 RGB FOVB ;
#define ASTRA_DEPTH_H 58.4
#define ASTRA_DEPTH_V 45.5

#define ASTRA_COLOR_H 63.1
#define ASTRA_COLOR_V 49.4  

// namespace  
using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

//指定time_t类型的时间，格式化为YYYYMMDDHH24MISS型的字符串  
void FormatTime(time_t ttime, char *szTime)
{
	struct tm tm1;

#ifdef WIN32  
	tm1 = *localtime(&ttime);
#else  
	localtime_r(&time1, &tm1);
#endif  
	sprintf(szTime, "%4.4d%2.2d%2.2d%2.2d%2.2d%2.2d",
		tm1.tm_year + 1900, tm1.tm_mon + 1, tm1.tm_mday,
		tm1.tm_hour, tm1.tm_min, tm1.tm_sec);
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(0.0, 0.0, 0.0);   //设置背景颜色
}

void CheckOpenNIError(Status result, string status)
{
	if (result != STATUS_OK)
		cerr << status << " Error: " << OpenNI::getExtendedError() << endl;
}

int main(int argc, char **argv)
{
#pragma region OpenCV PCL_initialization

	int i, j;
	cv::Point3f PointDepth(0.0, 0.0, 0.0);
	cv::Point3f PointWorld(0.0, 0.0, 0.0);

	IplImage *pImgBGR;

	//point cloud   
	PointCloud<PointXYZRGB> cloud;

	//opencv image  
	Mat cvBGRImg;
	Mat cvDepthImg;

	namedWindow("depth");
	namedWindow("image");

	//cloud for show
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZ>);
	//color cloud for show
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show_Color(new pcl::PointCloud<pcl::PointXYZRGB>);

	//创建viewer对象
	//pcl::visualization::CloudViewer viewerColor("Color Cloud Viewer"); 
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	char namePCD[256] = "test_pcdc.pcd";
	char nameColor[256] = "c_color.jpg";
	char nameDepth[256] = "c_depth.jpg";
	char nowTime[256] = "00000000000000";
	//char strfilepath[256] = "test_pcdc_segmented.pcd";
#pragma endregion

#pragma region openNI_initialization

	//OpenNI2 image    
	VideoFrameRef oniDepthImg;
	VideoFrameRef oniColorImg;

	Status result = STATUS_OK;

	// 初始化OpenNI    
	result = OpenNI::initialize();
	CheckOpenNIError(result, "initialize context");

	// open device      
	Device device;
	result = device.open(openni::ANY_DEVICE);
	CheckOpenNIError(result, "open device");

	VideoStream oniDepthStream;
	VideoMode modeDepth;
	VideoStream oniColorStream;
	VideoMode modeColor;
	if (STATUS_OK == result)
	{
		// create depth stream     
		result = oniDepthStream.create(device, openni::SENSOR_DEPTH);
		CheckOpenNIError(result, "create depth stream");

		// set depth video mode    
		
		modeDepth.setResolution(640, 480);//orbbec
										  //modeDepth.setResolution(512, 424);//kincet
		modeDepth.setFps(30);
		modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
		oniDepthStream.setVideoMode(modeDepth);

		// create color stream    
		
		result = oniColorStream.create(device, openni::SENSOR_COLOR);
		CheckOpenNIError(result, "create color stream");

		// set color video mode    
		
		modeColor.setResolution(640, 480);//orbbec
										  //modeColor.setResolution(1920, 1080);//kincet
		modeColor.setFps(30);
		modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
		oniColorStream.setVideoMode(modeColor);

		// 设置深度图像映射到彩色图像
		if (device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			result = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
			CheckOpenNIError(result, "setImageRegistrationMode");
		}
		result = device.setDepthColorSyncEnabled(TRUE);
		CheckOpenNIError(result, "setImageRegistrationMode");

		// start color stream    
		result = oniColorStream.start();
		CheckOpenNIError(result, "start color stream");
		// start depth stream    
		result = oniDepthStream.start();
		CheckOpenNIError(result, "start depth stream");
	}
	

	

	
#pragma endregion

	while(true)
	{
		// help
		if (cv::waitKey(1) == 'h')
		{
			cout << "*********************************************************" << endl;
			cout << "h ———— help" << endl;
			cout << "q ———— quit" << endl;
			cout << "r ———— read ColorStream and DepthStream" << endl;
			cout << "c ———— capture color.jpg depth.jpg and save test_pcdc.pcd " << endl;
			cout << "p ———— show point cloud" << endl;
			cout << "s ———— show color point cloud" << endl;
			cout << "d ———— show depth and color image" << endl;
			cout << "*********************************************************" << endl;
		}

		// quit  
		if (cv::waitKey(1) == 'q')
		{
			cout << "quit ……" << endl;
			break;
		}

		//read and show frame 
		if (cv::waitKey(1) == 'r')
		{
			// read frame    
			if (oniColorStream.readFrame(&oniColorImg) == STATUS_OK)
			{
				// convert data into OpenCV type    
				Mat cvRGBImg(oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData());
				cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
				imshow("image", cvBGRImg);
			}

			if (oniDepthStream.readFrame(&oniDepthImg) == STATUS_OK)
			{
				Mat cvRawImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());
				cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / (oniDepthStream.getMaxPixelValue()));
				imshow("depth", cvDepthImg);
			}
		}
			
		// capture  depth and color data     
		if (cv::waitKey(1) == 'c')
		{
			//get data  
			DepthPixel *pDepth = (DepthPixel*)oniDepthImg.getData();
			pImgBGR = &IplImage(cvBGRImg);
			//create point cloud  
			cloud.width = oniDepthImg.getWidth();
			cloud.height = oniDepthImg.getHeight();
			cloud.is_dense = false;
			cloud.points.resize(cloud.width * cloud.height);

			for (i = 0; i<oniDepthImg.getHeight(); i++)
			{
				for (j = 0; j<oniDepthImg.getWidth(); j++)
				{
					PointDepth.x = j;
					PointDepth.y = i;
					PointDepth.z = pDepth[i*oniDepthImg.getWidth() + j];
					//orbbec 利用视场角计算点云数据
					PointWorld.x = (PointDepth.x / 640.0 - 0.5)*(tan(ASTRA_COLOR_H*PI / 360.0) * 2 * PointDepth.z);
					PointWorld.y = (PointDepth.y / 480.0 - 0.5)*(tan(ASTRA_COLOR_V*PI / 360.0) * 2 * PointDepth.z);
					PointWorld.z = PointDepth.z;

					if ((PointWorld.z < 1300) && (PointWorld.x < 300))//右手坐标系
					{
						cloud[i*cloud.width + j].x = PointWorld.x/1000; //单位转成mm
						cloud[i*cloud.width + j].y = PointWorld.y/1000;
						cloud[i*cloud.width + j].z = PointWorld.z/1000;
						cloud[i*cloud.width + j].b = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 0];
						cloud[i*cloud.width + j].g = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 1];
						cloud[i*cloud.width + j].r = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 2];//手动添加偏移量10
					}
					/*cloud[i*cloud.width + j].x = PointWorld.x;
					cloud[i*cloud.width + j].y = PointWorld.y;
					cloud[i*cloud.width + j].z = PointWorld.z;
					cloud[i*cloud.width + j].b = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 0];
					cloud[i*cloud.width + j].g = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 1];
					cloud[i*cloud.width + j].r = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 2];*/
				}
			}
			
			//for (i = 0; i<oniDepthImg.getHeight(); i++)
			//{
			//	for (j = 0; j<oniDepthImg.getWidth(); j++)
			//	{
			//		PointDepth.x = i;
			//		PointDepth.y = j;
			//		PointDepth.z = pDepth[i*oniDepthImg.getWidth() + j];
			//		CoordinateConverter::convertDepthToWorld(oniDepthStream, PointDepth.x, PointDepth.y, PointDepth.z, &PointWorld.x, &PointWorld.y, &PointWorld.z);           //k,m左右手坐标系问题
			//		cloud[i*cloud.width + j].x = PointWorld.x;
			//		cloud[i*cloud.width + j].y = PointWorld.y;
			//		cloud[i*cloud.width + j].z = PointWorld.z;
			//		cloud[i*cloud.width + j].b = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + j * 3 + 0];
			//		cloud[i*cloud.width + j].g = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + j * 3 + 1];
			//		cloud[i*cloud.width + j].r = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + j * 3 + 2];
			//	}
			//}

			time_t nowtime;
			nowtime = time(NULL); //获取当前时间  
				
			FormatTime(nowtime, namePCD);
			strcat(namePCD, ".pcd");
			FormatTime(nowtime, nameColor);
			strcat(nameColor, "_color.jpg");
			FormatTime(nowtime, nameDepth);
			strcat(nameDepth, "_depth.jpg");

			pcl::io::savePCDFileBinaryCompressed(namePCD, cloud);//保存点云数据，以保存时间命名
			cerr << "Saved " << cloud.points.size() << " data points to " << namePCD << endl;

			imwrite(nameColor, cvBGRImg);
			imwrite(nameDepth, cvDepthImg);
			/*for(size_t i=0;i<cloud.points.size();++i)
			cerr<<"    "<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z<<endl;*/
		}

		//show point cloud
		if (cv::waitKey(1) == 'p')
		{
			
			if (-1 == pcl::io::loadPCDFile(namePCD, *cloud_show)) {
				cout << "cloud error input!" << endl;
				return -1;
			}
			cout << "showCloud Points:" << cloud_show->points.size() << endl;	

			viewer.showCloud(cloud_show);
			viewer.runOnVisualizationThreadOnce(viewerOneOff);
		}

		//show color point cloud
		if (cv::waitKey(1) == 's')
		{
			if (-1 == pcl::io::loadPCDFile(namePCD, *cloud_show_Color)) {
				cout << "cloud error input!" << endl;
				return -1;
			}
			cout << "showColorCloud Points:" << cloud_show_Color->points.size() << endl;
			viewer.showCloud(cloud_show_Color);
			viewer.runOnVisualizationThreadOnce(viewerOneOff);
		}

		//show depth and color image
		if (cv::waitKey(1) == 'd')
		{
			cvBGRImg = imread(nameColor);
			cvDepthImg = imread(nameDepth);

			cout << "show c_image and c_depth" << endl;
			imshow("c_image", cvBGRImg);
			imshow("c_depth", cvDepthImg);
		}
	}

#pragma region Close_Device
	// 销毁彩色数据流和深度数据流
	oniColorStream.destroy();
	oniDepthStream.destroy();
	// 关闭设备
	device.close();
	// 关闭OpenNI环境
	OpenNI::shutdown();
#pragma endregion

	return 0;
}