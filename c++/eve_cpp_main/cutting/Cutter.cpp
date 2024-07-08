#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions

// #include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>
#include "Cutter.h"
#include <unistd.h>

using namespace std;

// class Cutter 
// {

    Cutter::Cutter(uint8_t pin1)
    {
        wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers
        cutPin = pin1;

        pinMode(cutPin, OUTPUT);

        maxCutAttempts = 1;
    }

    void Cutter::setAttempts(uint8_t newAttempts)
    {
        maxCutAttempts = newAttempts;
    }

    void Cutter::resetCut() {
        digitalWrite(cutPin, 0);
    }

    void Cutter::cutPlant()
    {
        digitalWrite(cutPin, 1);
        usleep(500000);
        digitalWrite(cutPin, 0);
        usleep(500000);
        // uint8_t cutAttempts = 0;
        // bool cutSuccess = 0;

        // while(cutAttempts < maxCutAttempts && !cutSuccess)
        // {
        //     digitalWrite(cutPin, 1);
        //     delay(200);

        //     cutSuccess = checkCut();
        //     cutAttempts += 1;

        //     digitalWrite(cutPin, 0);
            
        // }
    }


    double Cutter::distance(float* pt1, float* pt2) {
        return sqrt(pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2));
    }

    // Helper function to calculate the mean of a vector
    // double Cutter::mean(const vector<int>& v) {
    //     return accumulate(v.begin(), v.end(), 0.0) / v.size();
    // }
    

    bool Cutter::fiducials() 
    {
        return true;
        // cv::Mat brighter;
        // frame.convertTo(brighter, -1, 3, 10);
        // cv::Mat hsv = frame;

        // Point pt1(0, 0);
        // Point pt2(0, 0);

        // int height = frame.rows;
        // int width = frame.cols;
        // int roi_start = static_cast<int>(0.5 * width);
        // cv::Mat roi = hsv(Rect(roi_start, 0, width - roi_start, height));

        // Scalar lower_red(0, 0, 100);
        // Scalar upper_red(70, 70, 255);

        // cv::Mat mask;
        // inRange(roi, lower_red, upper_red, mask);

        // vector<int> v_indices_r, u_indices_r;
        // for (int y = 0; y < mask.rows; y++) {
        //     for (int x = 0; x < mask.cols; x++) {
        //         if (mask.at<uchar>(y, x) != 0) {
        //             v_indices_r.push_back(y);
        //             u_indices_r.push_back(x);
        //         }
        //     }
        // }

        // if (!u_indices_r.empty()) {
        //     int avg_ur = static_cast<int>(mean(u_indices_r) + (0.5 * width));
        //     int avg_vr = static_cast<int>(mean(v_indices_r));
        //     pt1 = Point(avg_ur, avg_vr);
        // }

        // // left
        // Mat roi_l = hsv(Rect(0, 0, roi_start, height));
        // Mat mask_l;
        // inRange(roi_l, lower_red, upper_red, mask_l);

        // vector<int> v_indices_l, u_indices_l;
        // for (int y = 0; y < mask_l.rows; y++) {
        //     for (int x = 0; x < mask_l.cols; x++) {
        //         if (mask_l.at<uchar>(y, x) != 0) {
        //             v_indices_l.push_back(y);
        //             u_indices_l.push_back(x);
        //         }
        //     }
        // }

        // if (!u_indices_l.empty()) {
        //     int avg_ul = static_cast<int>(mean(u_indices_l) + static_cast<int>(width / 10));
        //     int avg_vl = static_cast<int>(mean(v_indices_l));
        //     pt2 = Point(avg_ul, avg_vl);
        // }

        // int red_dist_avg = static_cast<int>(distance(pt1, pt2));

        // int cut_complete;
        // if (red_dist_avg < 250) {
        //     cut_complete = 1;
        // } else {
        //     cut_complete = 0;
        // }

        // return cut_complete;
    }
// }