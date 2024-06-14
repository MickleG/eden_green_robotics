#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>


int resolution[2] = {424, 240};
int cropped_gripper_bounds[2] = {130, 400};

using namespace cv;
using namespace std;


int main() {
    cv::Mat original_rgb_image = cv::imread("/home/edengreen/eden_green_robotics/ideal_cutting_position_plant.png", cv::IMREAD_COLOR);
    //cv::imshow("original image", original_rgb_image);
    //cv::waitKey(0);

    while (true) {

        cv::Mat croppingMask(cv::Size(resolution[0], resolution[1]), CV_8UC1);
        
        cv::Rect roi(cropped_gripper_bounds[0], 0, cropped_gripper_bounds[1] - cropped_gripper_bounds[0], resolution[1]);
        croppingMask(roi) = 255;
        cv::Mat rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC1);
        cv::bitwise_and(original_rgb_image, original_rgb_image, rgb_image, croppingMask);

        cv::Mat hsv_image;
        cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV, 0);

        // Creating Mask using HSV thresholding
        cv::Mat cupMask(rgb_image.size(), CV_8UC1); // Match the size of the input image
        cv::Scalar lowerHSV(100, 200, 0);
        cv::Scalar upperHSV(150, 255, 255);

        cv::inRange(hsv_image, lowerHSV, upperHSV, cupMask);

        // Morphology to clean image
        cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
        cv::morphologyEx(cupMask, cupMask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(cupMask, cupMask, cv::MORPH_OPEN, kernel);

        // cv::imshow("cupMask", cupMask);

        int totalCupPixels = 0;
        float avgCupU = 0;
        float avgCupV = 0;

        for(int v = 0; v < cupMask.rows; v++) {
            for(int u = 0; u < cupMask.cols; u++) {
                if(cupMask.at<unsigned char>(v, u) == 255) {
                    avgCupU += u;
                    avgCupV += v;
                    totalCupPixels++;
                }
            }
        }

        

        cv::Point avgCupLocation(int(avgCupU / totalCupPixels), int(avgCupV / totalCupPixels));
        cv::circle(rgb_image, avgCupLocation, 5, cv::Scalar(255, 255, 0), -1);

        //cv::imshow("avgCupLocation", rgb_image);

        cout << "total cup points: " << totalCupPixels << " location is: " << avgCupLocation.x << ", " << avgCupLocation.y << endl;



        if (cv::waitKey(1) >= 0) break;
    }

    return 0;
}
