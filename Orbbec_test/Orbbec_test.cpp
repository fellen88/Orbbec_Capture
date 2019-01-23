// Orbbec_test.cpp : Defines the entry point for the console application.

// Ԥ����ͷ�ļ�
#include "stdafx.h"

// ��׼��ͷ�ļ�  
#include <iostream>  
#include <string>  
#include <vector>   

// PCLͷ�ļ�
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>

// OpenCVͷ�ļ�  
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>  

// OpenNIͷ�ļ�  
#include <OpenNI.h>   

typedef unsigned char uint8_t;

#define PI 3.141592689793

//1. ������ͼ���ɫû�������� ����ʽ�е� FOV Ӧʹ�� depth FOV ;
//2. ������ͼ���ɫͼ�������� ����ʽ�е� FOV Ӧʹ�� RGB FOVB ;
#define ASTRA_DEPTH_H 58.4
#define ASTRA_DEPTH_V 45.5

#define ASTRA_COLOR_H 63.1
#define ASTRA_COLOR_V 49.4  

// namespace  
using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

//ָ��time_t���͵�ʱ�䣬��ʽ��ΪYYYYMMDDHH24MISS�͵��ַ���  
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
	viewer.setBackgroundColor(0.0, 0.0, 0.0);   //���ñ�����ɫ
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

	//����viewer����
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

	// ��ʼ��OpenNI    
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

		// �������ͼ��ӳ�䵽��ɫͼ��
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
			cout << "h �������� help" << endl;
			cout << "q �������� quit" << endl;
			cout << "r �������� read ColorStream and DepthStream" << endl;
			cout << "c �������� capture color.jpg depth.jpg and save test_pcdc.pcd " << endl;
			cout << "p �������� show point cloud" << endl;
			cout << "s �������� show color point cloud" << endl;
			cout << "d �������� show depth and color image" << endl;
			cout << "*********************************************************" << endl;
		}

		// quit  
		if (cv::waitKey(1) == 'q')
		{
			cout << "quit ����" << endl;
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
					//orbbec �����ӳ��Ǽ����������
					PointWorld.x = (PointDepth.x / 640.0 - 0.5)*(tan(ASTRA_COLOR_H*PI / 360.0) * 2 * PointDepth.z);
					PointWorld.y = (PointDepth.y / 480.0 - 0.5)*(tan(ASTRA_COLOR_V*PI / 360.0) * 2 * PointDepth.z);
					PointWorld.z = PointDepth.z;

					if ((PointWorld.z < 1300) && (PointWorld.x < 300))//��������ϵ
					{
						cloud[i*cloud.width + j].x = PointWorld.x/1000; //��λת��mm
						cloud[i*cloud.width + j].y = PointWorld.y/1000;
						cloud[i*cloud.width + j].z = PointWorld.z/1000;
						cloud[i*cloud.width + j].b = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 0];
						cloud[i*cloud.width + j].g = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 1];
						cloud[i*cloud.width + j].r = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + (j - 10) * 3 + 2];//�ֶ����ƫ����10
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
			//		CoordinateConverter::convertDepthToWorld(oniDepthStream, PointDepth.x, PointDepth.y, PointDepth.z, &PointWorld.x, &PointWorld.y, &PointWorld.z);           //k,m����������ϵ����
			//		cloud[i*cloud.width + j].x = PointWorld.x;
			//		cloud[i*cloud.width + j].y = PointWorld.y;
			//		cloud[i*cloud.width + j].z = PointWorld.z;
			//		cloud[i*cloud.width + j].b = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + j * 3 + 0];
			//		cloud[i*cloud.width + j].g = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + j * 3 + 1];
			//		cloud[i*cloud.width + j].r = (uint8_t)pImgBGR->imageData[i*pImgBGR->widthStep + j * 3 + 2];
			//	}
			//}

			time_t nowtime;
			nowtime = time(NULL); //��ȡ��ǰʱ��  
				
			FormatTime(nowtime, namePCD);
			strcat(namePCD, ".pcd");
			FormatTime(nowtime, nameColor);
			strcat(nameColor, "_color.jpg");
			FormatTime(nowtime, nameDepth);
			strcat(nameDepth, "_depth.jpg");

			pcl::io::savePCDFileBinaryCompressed(namePCD, cloud);//����������ݣ��Ա���ʱ������
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
	// ���ٲ�ɫ�����������������
	oniColorStream.destroy();
	oniDepthStream.destroy();
	// �ر��豸
	device.close();
	// �ر�OpenNI����
	OpenNI::shutdown();
#pragma endregion

	return 0;
}