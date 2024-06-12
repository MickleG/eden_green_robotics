// Coordinate frame: Positive x is towards robot's left hand side, positive y is robot's up


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
#include <unistd.h>

#include <EndEffectorConfig.h>
#include <MotorConfig.h>
#include "Grip.h"
//#include <Cutter.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

std::atomic<bool> running(true);
std::atomic<int> goalZ(-1);
std::atomic<int> xOffset(0);
std::atomic<bool> initialCenteringDone(false);
std::atomic<bool> blueDetected(false);
std::atomic<bool> harvesting(false);

int hardware_buffer = 50;
int z_deadband_buffer = 15;
int x_deadband_buffer = 5;

int harvestToRibDist = 175; // distance from harvesting zone to when front vine becomes straight again in mm
int harvestToHarvestDist = 273; // distance between each harvesting zone in mm

bool gripping = false;
bool findingZ = false;

int resolution[2] = {424, 240};

int cropped_gripper_bounds[2] = {130, 400};

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
        int num_min_points = 2000; // Number of z-points to collect for average minimum depth sensing
        
        // These values were determined from bringing end effector to target cutting position and measuring on a static image
        int targetCupV = 220;
        int targetCupPointCount = 6043;

        // These values determined through testing
        int cupVThreshold = 10;
        int cupPixelThreshold = 2000;

        while (running) {
            // Wait for the next set of frames from the camera
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame().as<rs2::depth_frame>();

            // Convert RealSense frame to OpenCV matrix
            cv::Mat original_rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat hsv_image;
            cv::Mat depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            cv::Mat croppingMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

            // Cropping image to remove gripper fingers
            cv::Rect roi(cropped_gripper_bounds[0], 0, cropped_gripper_bounds[1] - cropped_gripper_bounds[0], resolution[1]);
            croppingMask(roi) = 255;
            cv::Mat rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::bitwise_and(original_rgb_image, original_rgb_image, rgb_image, croppingMask);


            // Changing color space from BGR to HSV, allowing for more robust color detection
            cv::cvtColor(rgb_image, hsv_image, COLOR_BGR2HSV, 0);


            // Creating Mask using HSV thresholding
            cv::Mat cupMask(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::Scalar lowerHSV(100, 200, 0);
            cv::Scalar upperHSV(150, 255, 255);
            cv::inRange(hsv_image, lowerHSV, upperHSV, cupMask);

            // Morphology to clean image of noise
            cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
            cv::morphologyEx(cupMask, cupMask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(cupMask, cupMask, cv::MORPH_OPEN, kernel);

            // cv::imshow("cupMask", cupMask);


            // Finding average location of cup and counting pixels in order to see if we arrived at harvesting location
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

            // cout << "blue data: " << abs(int(avgCupV / totalCupPixels) - targetCupV) << " and " << abs(totalCupPixels - targetCupPointCount) << endl;

            if(!harvesting) {
                if(abs(int(avgCupV / totalCupPixels) - targetCupV) <= cupVThreshold && abs(totalCupPixels - targetCupPointCount) <= cupPixelThreshold) {
                    blueDetected = true;
                    // cout << "blue detected! and initialCenteringDone: " << initialCenteringDone << endl;
                } else {
                    blueDetected = false;
                    // cout << "blue NOT detected" << endl;
                }
            }

            // Creating vineMask and depthImage to use for visual servoing in x and z, respectively

            cv::Mat vineMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::Mat depthImage = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_64F);
            

            // Extracting vine using distance away from camera (cutting away far background points greater than 0.3m away) and HSV color thresholding (cutting away color mismatches from the vine)
            // Additional check val != 0 is added due to points lying outside of Realsense detection zone being labelled as distance = 0
            for (int v = 0; v < depth_image.rows; v++) {
                for (int u = 0; u < depth_image.cols; u++) {
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


            // // Removing cups from vineMask to improve vine masking (as the gray color on the vines is a blue-based hue)
            cv::subtract(vineMask, cupMask, vineMask);

            // More morphology to clean vineMask of noise
            cv::morphologyEx(vineMask, vineMask, cv::MORPH_OPEN, kernel);


            // Collecting locations of n lowest depth points (n = 1000), adding to mask called smallest_values.
            // This mask will then average the distances of all n points in order to get a more robust estimation for
            // the minimum distance the vine is away from the end effector
            std::vector<PointValue> points;

            for(int v = 0; v < vineMask.rows; v++) {
                for(int u = 0; u < vineMask.cols; u++) {
                    float val = depthImage.at<double>(v, u);

                    if (vineMask.at<unsigned char>(v, u) != 0 && val != 0) {
                        points.push_back({val, v, u});
                    }
                    
                }
            }

            // Sorting points from lowest to highest value, then choosing the smallest n from that sorted list
            std::sort(points.begin(), points.end(), [](const PointValue& a, const PointValue& b) {
                return a.value < b.value;
            });

            std::vector<PointValue> smallest_z_values(points.begin(), points.begin() + std::min(num_min_points, static_cast<int>(points.size())));



            // Processing to find the vertical section of the smallest_values mask, corresponding to the front rib of the vine
            Mat smallest_values_raw = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

            for(const auto& point : smallest_z_values) {
                smallest_values_raw.at<unsigned char>(point.v, point.u) = 255;
            }

            // Running morphology to clean up noise from smallest_values mask
            cv::morphologyEx(smallest_values_raw, smallest_values_raw, cv::MORPH_OPEN, kernel);


            // Using a tall and thin kernel in order to extract similar tall and thin features from the image.
            // This extracts the front rib of the vine, which is what we want to servo off of
            cv::Mat verticalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 20));
            cv::Mat filtered_smallest_values_raw;
            cv::erode(smallest_values_raw, smallest_values_raw, verticalStructure);
            cv::dilate(smallest_values_raw, smallest_values_raw, verticalStructure);

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;

            cv::findContours(smallest_values_raw, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Find the largest contour which should be the front vine rib
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

            cv::imshow("smallest_values_filtered", smallest_values_filtered);

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

            cv::Point minLoc(min_z_u, min_z_v);

            goalZ = minVal * 1000;


            int avg_u = int((max_u + min_u) / 2);

            xOffset = int(resolution[0] / 2) - avg_u;

            cv::Point centerX(avg_u, int(resolution[1] / 2));

            cv::circle(rgb_image, minLoc, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(rgb_image, centerX, 5, cv::Scalar(0, 0, 255), -1);

            cv::imshow("min z point and avg x", rgb_image);

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
    int xServoingSpeed = -1;
    int zServoingSpeed = -1;
    float zOffset = 0;
    int centeredCounter = -1;
    // bool x_centered = false;
    // bool z_centered = false;

    while (running) {

        if(goalZ >= 0 && !blueDetected) { // initialized to -1, this check is to ensure image_processing gives valid goalZ before movement
            
            if(!findingZ) {
                if(abs(xOffset) < x_deadband_buffer) {
                    xServoingSpeed = 0;
                    findingZ = true;
                    // x_centered = true;
                } else {
                    xServoingSpeed = xOffset;
                    // x_centered = false;
                }

                mechanism.moveInX(-xServoingSpeed);
                mechanism.updateCurrentPosition();
            }

            if (findingZ) { 
                zOffset = goalZ - hardware_buffer;

                if(abs(zOffset) < z_deadband_buffer) {
                    zServoingSpeed = 0;
                    findingZ = false;
                    // z_centered = true;
                } else {
                    zServoingSpeed = zOffset;
                    // z_centered = false;
                }

                mechanism.moveInZ(zServoingSpeed);
                mechanism.updateCurrentPosition();
            }
            if(xServoingSpeed == 0 && zServoingSpeed == 0) {
                centeredCounter++;
                if(centeredCounter >= std::numeric_limits<int>::max()) {
                    centeredCounter = 1;
                }
            }
        }

        if(centeredCounter == 0) {
            initialCenteringDone = true;
        }


    }
}


// main function
int main() {   
    // INITIALIZE ALL MOTORS AND OBJECTS//

    // int curLimit = 90;
    // int goalCur = 200;

    // //Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent, min angle, max angle)
    // MotorXM430 servo1(1, 5, curLimit, goalCur, 225, 315);
    // MotorXM430 servo2(2, 5, curLimit, goalCur, 315, 225);

    
    EndEffectorConfig mechanism(0, 0); // defines the positioning mechanism

    // //Accessing the motors function
    // servo1.PrintOperatingMode();
    // servo2.PrintOperatingMode();

    // servo1.SetProfile(32000, 400); // have to be in velocity mode I think
    // servo2.SetProfile(32000, 400); // have to be in velocity mode I think

    // // Open grippers at startup
    // drop(servo1, servo2);


    mechanism.calibrateZero(100);
    mechanism.updateCurrentPosition();

    cout << "current Z: " << mechanism.zPosition << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    cout << "Moving in Z a little to make room in X" << endl;


    mechanism.goToPosition(0, 200, 100);
    mechanism.updateCurrentPosition();

    cout << "currentPosition: " << mechanism.xPosition << ", " << mechanism.yPosition << ", " << mechanism.zPosition << endl;


    cout << "current Z: " << mechanism.zPosition << endl;
    cout << "Ready to Harvest" << endl;
    
    // std::thread image_processing_thread(image_processing);
    while(true) {
        // cout << "going to 50" << endl;
        mechanism.goToPosition(0, 100, 100);
        mechanism.updateCurrentPosition();
        // cout << "currentPosition: " << mechanism.xPosition << ", " << mechanism.yPosition << ", " << mechanism.zPosition << endl;
        usleep(5000000);
        // cout << "going to 200" << endl;
        mechanism.goToPosition(0, 200, 100);
        mechanism.updateCurrentPosition();
        // cout << "currentPosition: " << mechanism.xPosition << ", " << mechanism.yPosition << ", " << mechanism.zPosition << endl;
        usleep(500000);
    }
    // std::thread motor_thread(visual_servoing, mechanism);

    // mechanism.yMotor.setSpeed(50);
    // mechanism.yMotor.setAcceleration(50);

    // while(running) {
    //     if(initialCenteringDone && !blueDetected) {
    //         mechanism.yMotor.motorDriveY();
    //         mechanism.updateCurrentPosition();
    //     } else if (blueDetected) {
    //         harvesting = true;
    //         cout << "gripping and cutting" << endl;

    //         grip(servo1, servo2);
    //         usleep(500000);
    //         float currentX = mechanism.xPosition;
    //         float currentY = mechanism.yPosition;
    //         float currentZ = mechanism.zPosition;

    //         cout << "currentPosition: " << currentX << ", " << currentY << ", " << currentZ << endl;

    //         cout << "dropping" << endl;

    //         mechanism.goToPosition(0, 100, 100);
    //         drop(servo1, servo2);
    //         usleep(500000);

    //         cout << "going back to harvest zone" << endl;
    //         mechanism.goToPosition(currentX, currentZ, 100);

    //         cout << "raising up" << endl;

    //         while(abs(mechanism.yPosition - currentY) <= 175) {
    //             mechanism.yMotor.motorDriveY();
    //             mechanism.updateCurrentPosition();
    //         }

    //         blueDetected = false;

    //     }
    // }
    
    // image_processing_thread.join();
    // motor_thread.join();


    return 0;
}
