//
// Created by ljy on 2021/2/1.
//

#ifndef SRC_POSECALCULATOR_H
#define SRC_POSECALCULATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <string>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include "UWB_message_type.h"
#include <nlink_parser/LinktrackNodeframe0.h>
#include <nlink_parser/LinktrackNodeframe3.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackNode0.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "Kalman.h"

class PoseCalculator {
public:
    std::vector<double> distcoeffs;
    cv::Mat cameraMatrix;
    cv::Mat contoursImg;
    std::string showInfo = "";
    cv::Mat last_rvec, last_tvec;
    bool last_vec_useful = false;
    bool calculate(cv::Mat& image, cv::Mat& rvec, cv::Mat& tvec, int pnp_method_flag = 0)
    {
        //统计图案的大小
        int bright_pixel_threshold = 30;
        int bright_pixel_count = 0;
        for(int i = 0; i < image.rows; i++)
            for(int j = 0; j < image.cols; j++)
            {
                int pixel_value = image.data[i * image.cols + j];
                if(pixel_value > bright_pixel_threshold)
                    bright_pixel_count++;
            }
        // cv::Mat testimage;
        // cv::threshold(image, testimage, 30, 255, cv::THRESH_BINARY);
        // cv::putText(testimage, ">30: " + std::to_string(bright_pixel_count),
        //                 cv::Point(100,100), cv::FONT_HERSHEY_PLAIN, 2, 255, 2);
        // cv::imshow("bin30",testimage);
        // ROS_WARN_STREAM("bright_pixel_count " << bright_pixel_count);

        //锐化
        cv::Mat blur_img, sharpen_img;
        double weight_sharpen = 0.6; // 0.1~0.9
        double sigma = std::max(2.0, std::min(12.0, std::sqrt(bright_pixel_count)/10));
        cv::GaussianBlur(image, blur_img, cv::Size(45, 45), sigma);  // Size越大，耗资源越多。
        cv::addWeighted(image, 1/(1-weight_sharpen), blur_img, -weight_sharpen/(1-weight_sharpen), 0, sharpen_img);


        cv::Mat binImg(sharpen_img.size(), CV_8UC1);
        // cv::adaptiveThreshold(sharpen_img, binImg, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 101, 20);
        cv::threshold(sharpen_img, binImg, 30, 255, cv::THRESH_TOZERO);
        cv::threshold(binImg, binImg, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;       //0 后一个轮廓的序号, 1 前一个轮廓的序号, 2 子轮廓的序号, 3 父轮廓的序号

        cv::findContours(binImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        cv::cvtColor(image, contoursImg, cv::COLOR_GRAY2BGR);
        cv::drawContours(contoursImg, contours, -1, cv::Scalar(0, 0, 255), 1, 8, hierarchy);

        std::vector<int> key_contours_indexes;
        for(int contour_index = 0; contour_index < contours.size(); contour_index++)
        {
            std::vector<int> same_level_contours = getSameLevelContour(hierarchy, contour_index);
            if(same_level_contours.size() != 3)
                continue;
            int parent_index = hierarchy[contour_index][3];
            //若没有父轮廓
            if(parent_index == -1)
                continue;
            //同级轮廓没有子轮廓
            for(int i = 0; i < same_level_contours.size();i++)
            {
                if(hierarchy[i][2] != -1) //若有子轮廓
                    continue;
            }
            std::vector<int> parent_level_contours = getSameLevelContour(hierarchy, parent_index);
            //parent同级两个轮廓
            if(parent_level_contours.size() != 2)
                continue;
            int parent_level_another_contour = parent_level_contours[1];
            int cousin_index = hierarchy[parent_level_another_contour][2];
            if( cousin_index == -1)
                continue;
            //cousin 应该无前无后无子
            if(hierarchy[cousin_index][0] != -1 || hierarchy[cousin_index][1] != -1 || hierarchy[cousin_index][2] != -1)
                continue;
            key_contours_indexes = same_level_contours;
            key_contours_indexes.insert(key_contours_indexes.begin(), cousin_index);
        }

        if(key_contours_indexes.size() == 4)
        {
            std::vector<cv::Point2d> key_points_unsorted(key_contours_indexes.size());
            double x_sum = 0, y_sum = 0;
            for(int i = 0; i < key_contours_indexes.size(); i++)
            {
                cv::Point2d contour_center = getCentroid(contours[key_contours_indexes[i]], image);
                x_sum += contour_center.x;
                y_sum += contour_center.y;
                key_points_unsorted[i] = contour_center;
            }
            cv::Point2d center_point(x_sum/key_contours_indexes.size(), y_sum/key_contours_indexes.size());
            std::vector<double> angle_to_center(key_contours_indexes.size());
            std::vector<int> key_point_index;
            for(int i = 0; i < key_contours_indexes.size(); i++)
            {
                key_point_index.push_back(i);
                angle_to_center[i] = std::atan2(key_points_unsorted[i].y - center_point.y, key_points_unsorted[i].x - center_point.x);
            }
            std::sort(key_point_index.begin(), key_point_index.end(),
                      [&](int a, int b){return angle_to_center[a] < angle_to_center[b];});
            int start_index = -1;
            for(int i = 0; i < key_point_index.size();i++)
            {
                if(key_point_index[i] == 0)
                {
                    start_index = i;
                    break;
                }
            }
            std::vector<cv::Point2d> key_points(key_contours_indexes.size());
            for(int i = 0; i < key_point_index.size();i++)
            {
                key_points[i] = key_points_unsorted[key_point_index[(start_index + i)%key_point_index.size()]];
                cv::putText(contoursImg, std::to_string(i) , key_points[i], cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255,120,0), 3);
            }

            //左上角开始 顺时针 前x 左y 上z
            std::vector<cv::Point3f> objectPts;
            objectPts.emplace_back(-150, 55, -10);
            objectPts.emplace_back(-150, -55, -10);
            objectPts.emplace_back(-150, -55, -50);
            objectPts.emplace_back(-150, 55, -50);

            if(last_vec_useful)
            {
                rvec = last_rvec;
                tvec = last_tvec;
                last_vec_useful = cv::solvePnP(objectPts, key_points, cameraMatrix, distcoeffs, rvec, tvec, true, pnp_method_flag);
            } else
            {
                last_vec_useful = cv::solvePnP(objectPts, key_points, cameraMatrix, distcoeffs, rvec, tvec, false, pnp_method_flag);
            }
        }
        last_vec_useful = false;
        return last_vec_useful;
    }
    void visualize()
    {
        cv::putText(contoursImg, showInfo,
                         cv::Point(100,100), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0,0,255), 2);
        cv::imshow("contoursImg", contoursImg);
    }
    void addShowString(std::string string_to_show)
    {
        showInfo += string_to_show + "\n";
    }
    void clearShowInfo()
    {
        showInfo = "";
    }
private:
    std::vector<int> getSameLevelContour(std::vector<cv::Vec4i> &hierarchy, int index)
    {
        if(index < 0 || index >= hierarchy.size())
            return std::vector<int>();
        std::vector<int> result = {index};
        int next;
        next = hierarchy[index][0]; //前一个轮廓
        while(next != -1)
        {
            result.push_back(next);
            next = hierarchy[next][0];
        }
        next = hierarchy[index][1]; //后一个
        while(next != -1)
        {
            result.push_back(next);
            next = hierarchy[next][1];
        }
        return result;
    }

    cv::Point2d getCentroid(std::vector<cv::Point> contour, cv::Mat grayImage)
    {
        cv::Rect bbox = cv::boundingRect(contour);
        unsigned long long pixel_weight_sum = 0;
        unsigned long long pixel_x_sum = 0, pixel_y_sum = 0;
        for(int i = bbox.x; i <= bbox.x + bbox.width; i++)
        {
            for(int j = bbox.y; j <= bbox.y + bbox.height; j++)
            {
                cv::Point point(i,j);
                if(cv::pointPolygonTest(contour, point, false) >= 0)
                {
                    pixel_weight_sum += grayImage.at<uchar>(point);
                    pixel_x_sum += i * grayImage.at<uchar>(point);
                    pixel_y_sum += j * grayImage.at<uchar>(point);
                }
            }
        }
        return cv::Point2d(pixel_x_sum * 1.0/pixel_weight_sum, pixel_y_sum * 1.0/pixel_weight_sum);
    }
};


#endif //SRC_POSECALCULATOR_H
