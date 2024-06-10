#include <stdio.h> 
#include <stdlib.h>
#include <time.h>       // For NANOS function
#include <stdint.h>
#include <cmath>        // For sqrt and other math functions
#include <time.h>
#include <wiringPi.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <algorithm>

#include <EndEffectorConfig.h>
#include <MotorConfig.h>
//#include <XM430.h>
//#include <Cutter.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

std::atomic<bool> running(true);
std::atomic<int> goalZ(-1);
std::atomic<int> xOffset(0);

int hardware_buffer = 50;
int deadband_buffer = 15;

bool gripping = false;
bool findingZ = false;

int resolution[2] = {424, 240};

struct PointValue {
    float value;
    int v;
    int u;
};

uint64_t getNanos()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * (uint64_t)1000000000 + (uint64_t)(ts.tv_nsec);
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void image_processing() {
    cout << "image processing running" << endl;
    try {
        rs2::pipeline pipe;
        rs2::config cfg;

        // Configure and start the pipeline
        cfg.enable_stream(RS2_STREAM_COLOR, resolution[0], resolution[1], RS2_FORMAT_BGR8, 15);
        cfg.enable_stream(RS2_STREAM_DEPTH, resolution[0], resolution[1], RS2_FORMAT_Z16, 15);
        pipe.start(cfg);

        rs2::align align_to_color(RS2_STREAM_COLOR);

        // int avg_u = 0;
        int num_min_points = 1000;

        while (running) {
            // Wait for the next set of frames from the camera
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame().as<rs2::depth_frame>();

            // Convert RealSense frame to OpenCV matrix
            cv::Mat rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat hsv_image;
            cv::Mat depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            cv::cvtColor(rgb_image, hsv_image, COLOR_BGR2HSV, 0);


            // Creating Mask using HSV thresholding
            cv::Mat cupMask(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::Scalar lowerHSV(100, 200, 0);
            cv::Scalar upperHSV(150, 255, 255);

            cv::inRange(hsv_image, lowerHSV, upperHSV, cupMask);

            // Morphology to clean image
            cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
            cv::morphologyEx(cupMask, cupMask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(cupMask, cupMask, cv::MORPH_OPEN, kernel);

            // cv::imshow("mask", cupMask);

            cv::Mat vineMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

            // Depth sensing across a cropped ROI to find nonzero minZ as well as to isolate vine geometry for masking
            // rows is resolution[1], cols is resolution[0]

            cv::Mat depthImage = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_64F);
            

            // Extracting vine using distance away from camera (cutting away far background points) and HSV color thresholding (cutting away color mismatches from the vine)
            for (int v = 0; v < depth_image.rows - 60; v++) {
                for (int u = 70; u < depth_image.cols - 70; u++) {
                    float val = depth_frame.get_distance(u, v);
                    depthImage.at<double>(v, u) = val;
                    cv::Vec3b hsv_pixel = hsv_image.at<cv::Vec3b>(v, u);

                    bool hue_condition = 90 <= hsv_pixel[0] && 130 >= hsv_pixel[0];
                    bool sat_condition = 0 <= hsv_pixel[1] && 200 >= hsv_pixel[1];
                    bool val_condition = 0 <= hsv_pixel[2] && 255 >= hsv_pixel[2];

                    if (val < 0.3 && hue_condition && sat_condition && val_condition && val != 0) {
                        vineMask.at<unsigned char>(v, u) = 255;
                    }

                }
            }

            
            // cout << "offset is: " << goalZ - hardware_buffer << " servoingSpeed is: " << servoingSpeed << endl;

            // // Removing cups from vineMask to improve vine masking (as the gray color on the vines is a blue-based hue)
            cv::subtract(vineMask, cupMask, vineMask);

            // cv::imshow("vineMask before morph", vineMask);

            cv::Mat kernel_small = cv::Mat::ones(5, 5, CV_8U);
            cv::morphologyEx(vineMask, vineMask, cv::MORPH_OPEN, kernel_small);

            // cv::imshow("vineMask after morph", vineMask);


            // collecting locations of n lowest depth points (n = 100), adding to mask called smallest_values
            std::vector<PointValue> points;

            for(int v = 0; v < vineMask.rows; v++) {
                for(int u = 0; u < vineMask.cols; u++) {
                    float val = depthImage.at<double>(v, u);

                    if (vineMask.at<unsigned char>(v, u) != 0 && val != 0) {
                        points.push_back({val, v, u});
                    }
                    
                }
            }

            std::sort(points.begin(), points.end(), [](const PointValue& a, const PointValue& b) {
                return a.value < b.value;
            });

            std::vector<PointValue> smallest_z_values(points.begin(), points.begin() + std::min(num_min_points, static_cast<int>(points.size())));



            // Processing to find the vertical section of the smallest_values mask, corresponding to the front rib of the vine
            Mat smallest_values_raw = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

            for(const auto& point : smallest_z_values) {
                smallest_values_raw.at<unsigned char>(point.v, point.u) = 255;
            }

            // cv::imshow("smallest_values_raw before morph", smallest_values_raw);

            cv::morphologyEx(smallest_values_raw, smallest_values_raw, cv::MORPH_OPEN, kernel);

            cv::Mat verticalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 20));
            cv::Mat filtered_smallest_values_raw;
            cv::erode(smallest_values_raw, smallest_values_raw, verticalStructure);
            cv::dilate(smallest_values_raw, smallest_values_raw, verticalStructure);

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(smallest_values_raw, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Find the largest contour which should be the vertical blob
            int largestContourIndex = -1;
            double largestContourArea = 0.0;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > largestContourArea) {
                    largestContourArea = area;
                    largestContourIndex = static_cast<int>(i);
                }
            }

            cv::Mat smallest_values = cv::Mat::zeros(vineMask.size(), CV_8UC1);
            if (largestContourIndex != -1) {
                cv::drawContours(smallest_values, contours, largestContourIndex, cv::Scalar(255), cv::FILLED);
            }

            cv::Mat smallest_values_filtered;
            // extracting the points from original smallest_values_raw mask that lie within the idealized vertical mask
            cv::bitwise_and(smallest_values, smallest_values_raw, smallest_values_filtered);

            // cv::imshow("smallest_values_filtered", smallest_values_filtered);

            float min_z_u = 0;
            float min_z_v = 0;
            float minVal = 0;
            int average_counter = 0;

            int min_u = std::numeric_limits<int>::max();
            int max_u = 0;
            

            for (int v = 0; v < smallest_values_filtered.rows; v++) {
                for (int u = 0; u < smallest_values_filtered.cols; u++) {
                    if(smallest_values_filtered.at<unsigned char>(v, u) == 255) {
                        min_z_u += u;
                        min_z_v += v;
                        minVal += depthImage.at<double>(v, u);
                        average_counter++;

                        if(u > max_u) {
                            max_u = u;
                        }
                        if(u < min_u) {
                            min_u = u;
                        } 
                    }
                }
            }

            min_z_u = int(min_z_u / average_counter);
            min_z_v = int(min_z_v / average_counter);
            minVal = minVal / average_counter;

            // cout << "minVal: " << minVal << endl;

            cv::Point minLoc(min_z_u, min_z_v);

            

            goalZ = minVal * 1000;


            int avg_u = int((max_u + min_u) / 2);

            xOffset = int(resolution[0] / 2) - avg_u;

            // cout << "xOffset: " << xOffset << endl;

            cv::Point centerX(avg_u, int(resolution[1] / 2));

            cv::circle(rgb_image, minLoc, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(rgb_image, centerX, 5, cv::Scalar(0, 0, 255), -1);

            // cv::imshow("min z point and avg x", rgb_image);

            char c = (char)cv::waitKey(1);

            if(c == 'q') {
                running = false;
            }
        }

        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "Exception in image_processing: " << e.what() << std::endl;
    }
}

void visual_servoing(EndEffectorConfig mechanism) {
    cout << "motors running" << endl;
    int xServoingSpeed = 0;
    int zServoingSpeed = 0;
    float zOffset = 0;

    while (running) {

        if(goalZ >= 0) { // initialized to -1, this check is to ensure image_processing gives valid goalZ before movement
            
            if(!findingZ) {
                if(abs(xOffset) < deadband_buffer) {
                    xServoingSpeed = 0;
                    findingZ = true;
                } else {
                    xServoingSpeed = xOffset;
                }

                mechanism.moveInX(-xServoingSpeed);
            }

            if (findingZ) { 
                zOffset = goalZ - hardware_buffer;

                if(abs(zOffset) < deadband_buffer) {
                    zServoingSpeed = 0;
                    findingZ = false;
                } else {
                    zServoingSpeed = zOffset;
                }

                mechanism.moveInZ(zServoingSpeed);
                // mechanism.moveInZAccel(servoingSpeed, 500); // updateCurrentPosition() is called within this function
            }
        }
    }
}


// main function
int main() {   
    // INITIALIZE ALL MOTORS AND OBJECTS//
    EndEffectorConfig mechanism(0, 0); // defines the positioning mechanism

    // UPDATE Y STAGE PINS
    // // MotorConfig yStage(-1, -1, -1, -1); // defines the y stage motor, will need to determine if up/down is positive or negative speed

    mechanism.calibrateZero(100);
    mechanism.updateCurrentPosition();

    cout << "current Z: " << mechanism.zPosition << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    cout << "Moving in Z a little to make room in X" << endl;


    mechanism.goToPosition(0, 200, 100);
    mechanism.updateCurrentPosition();

    cout << "current Z: " << mechanism.zPosition << endl;
    cout << "Ready to Harvest" << endl;
    
    std::thread image_processing_thread(image_processing);
    std::thread motor_thread(visual_servoing, mechanism);
    
    image_processing_thread.join();
    motor_thread.join();


    return 0;
}
