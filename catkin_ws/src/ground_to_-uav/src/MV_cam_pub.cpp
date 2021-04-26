#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <string>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>



bool g_bExit = false;
unsigned int g_nPayloadSize = 0;
image_transport::Publisher image_pub;
ros::Publisher drone_center_pub;


// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
    g_bExit = true;
    sleep(1);
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}



static  void* WorkThread(void* pUser)
{
    int nRet = MV_OK;
    static int image_index = 0;


    MV_FRAME_OUT stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

    while(ros::ok())
    {
        nRet = MV_CC_GetImageBuffer(pUser, &stOutFrame, 1000);
        cv::Mat image = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC1, (void *)stOutFrame.pBufAddr);
        if (nRet == MV_OK)
        {
            //printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
              //  stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum);
            
            // std::vector<std::vector<cv::Point>> contours;
            // std::vector<cv::Vec4i> hierarchy;

            // cv::findContours(image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            
            // double largest_area = 0;
            // int largest_contour_index=0;
            // for( size_t i = 0; i< contours.size(); i++ ) // iterate through each contour.
            // {
            //     double area = cv::contourArea( contours[i] );  //  Find the area of contour
            //     if( area > largest_area )
            //     {
            //         largest_area = area;
            //         largest_contour_index = i;               //Store the index of largest contour
            //     }
            // }

            // cv::Moments mu = cv::moments( contours[largest_contour_index], false );
            // cv::Point2f drone_center( mu.m10/mu.m00 , mu.m01/mu.m00 );
            

            try{
                cv_bridge::CvImage cv_img;
                cv_img.header.stamp = ros::Time::now();
                cv_img.encoding = "mono8";
                cv_img.image = image;
                image_pub.publish(cv_img.toImageMsg());

            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            
            // cv::circle(image, drone_center, 2, cv::Scalar(0,255,255));
            cv::imshow("mv_image", image);

            ros::spinOnce();

            char key_board = cv::waitKey(1);
            if(key_board == 'w')
            {
                std::string file_name;
                file_name = file_name + "/home/ljy/cali_image/image";
                file_name = file_name + char(image_index + '0');
                file_name = file_name + ".jpg";
                cv::imwrite(file_name, image);
                std::cout << file_name << std::endl;
                image_index++;
            }
        }
        else
        {
            printf("No data[0x%x]\n", nRet);
        }
        if(NULL != stOutFrame.pBufAddr)
        {
            nRet = MV_CC_FreeImageBuffer(pUser, &stOutFrame);
            if(nRet != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
        }
        if(g_bExit)
        {
            break;
        }
    }

    return 0;
}

int main(int argc, char *argv[])
{
    int nRet = MV_OK;
    void* handle = NULL;

    ros::init(argc, argv, "MV_cam_pub_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("/MVcam/image", 1);
    drone_center_pub = nh.advertise<geometry_msgs::Vector3>("/drone/center", 1);


    do 
    {
        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Intput camera index:");
        unsigned int nIndex = 0;
        // scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        //设置参数
        nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 0);    // 0: Off
        if (MV_OK == nRet)
        {
            printf("set ExposureAuto OK!\n\n");
        }
        else
        {
            printf("set ExposureAuto failed! nRet [%x]\n\n", nRet);
        }
        
        float exposureTime = 20000.0f;
        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);    
        if (MV_OK == nRet)
        {
            printf("set ExposureTime OK!\n\n");
        }
        else
        {
            printf("set ExposureTime failed! nRet [%x]\n\n", nRet);
        }

        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 30.0f);    // 0: Off
        if (MV_OK == nRet)
        {
            printf("set AcquisitionFrameRate OK!\n\n");
        }
        else
        {
            printf("set AcquisitionFrameRate failed! nRet [%x]\n\n", nRet);
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            break;
        }
        g_nPayloadSize = stParam.nCurValue;

        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

		pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , handle);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            break;
        }

        PressEnterToExit();

        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
    } while (0);
    

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("exit.\n");

    return 0;
}
