// Coordinate frame: Positive speed for each motor is towards outer limit switch. Positive x is towards robot's right hand side, positive y is robot's up


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
#include <Cutter.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;

std::atomic<bool> running(true);
std::atomic<int> goalZ(-1);
std::atomic<int> xOffset(0);
std::atomic<float> v_avg(-1);
std::atomic<bool> initialCenteringDone(false);
std::atomic<bool> harvestZoneDetected(false);
std::atomic<bool> blueDetected(false);
std::atomic<bool> harvesting(false);
std::atomic<bool> liftingY(false);
// std::atomic<bool> startTrackingHeight(false);

int hardware_buffer = 35;
int z_deadband_buffer = 25;
int x_deadband_buffer = 5;

int harvestToRibDist = 175; // distance from harvesting zone to when front vine becomes straight again in mm
int harvestToHarvestDist = 273; // distance between each harvesting zone in mm

bool gripping = false;
bool findingZ = false;

int resolution[2] = {424, 240};

int cropped_gripper_bounds[2] = {130, 400};

bool runMotors = true;


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
        int num_min_cupPoints = 200;
        
        // These values were determined from bringing end effector to target cutting position and measuring on a static image
        int targetCupV = 220;
        int targetCupPointCount = 1000; //3452

        // These values determined through testing
        int cupVThreshold = 10;
        int cupPixelThreshold = 100; //1000

        int blueThresholdPixels = 100;
        int counter = 0;

        while (running) {
            // Wait for the next set of frames from the camera
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame().as<rs2::depth_frame>();

            // Convert RealSense frame to OpenCV matrix
            cv::Mat original_rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat hsv_image;
            cv::Mat original_depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            //cv::imshow("original_image", original_rgb_image);

            // Code for saving images
            // cv::imwrite("/home/edengreen/eden_green_robotics/c++/eve_cpp_main/images/rgb_image" + to_string(counter) + ".png", original_rgb_image);
            // cv::imwrite("/home/edengreen/eden_green_robotics/c++/eve_cpp_main/images/depth_image" + to_string(counter) + ".png", depth_image);

            cv::Mat croppingMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

            // Cropping image to remove gripper fingers
            cv::Rect roi(cropped_gripper_bounds[0], 0, cropped_gripper_bounds[1] - cropped_gripper_bounds[0], resolution[1]);
            croppingMask(roi) = 255;
            cv::Mat rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::Mat depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1);
            cv::bitwise_and(original_rgb_image, original_rgb_image, rgb_image, croppingMask);
            cv::bitwise_and(original_depth_image, original_depth_image, depth_image, croppingMask);



            // Changing color space from BGR to HSV, allowing for more robust color detection
            cv::cvtColor(rgb_image, hsv_image, COLOR_BGR2HSV, 0);

            // cv::imshow("hsv", hsv_image);


            // Creating Mask using HSV thresholding
            cv::Mat cupMask(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::Scalar lowerHSV(100, 200, 0);
            cv::Scalar upperHSV(150, 255, 255);
            cv::inRange(hsv_image, lowerHSV, upperHSV, cupMask);

            // cv::imshow("cupMask beforemorph", cupMask);

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

            // cv::imshow("cupMask", cupMask);
            

            // cout << "blue data: " << abs(int(avgCupV / totalCupPixels) - targetCupV) << " and " << abs(totalCupPixels - targetCupPointCount) << endl;

            if(!harvesting && initialCenteringDone) {

                if(totalCupPixels > 0) {
                    vector<int> cupPoints;

                    for(int v = 0; v < cupMask.rows; v++) {
                        for(int u = 0; u < cupMask.cols; u++) {
                            if(cupMask.at<unsigned char>(v, u) == 255) {
                                cupPoints.push_back(v);
                            }
                        }
                    }

                    std::sort(cupPoints.begin(), cupPoints.end(), [](const int& a, const int& b) {
                        return a > b;
                    });

                    std::vector<int> largest_v_values(cupPoints.begin(), cupPoints.begin() + std::min(num_min_cupPoints, static_cast<int>(cupPoints.size())));

                    v_avg = 0;
                    float v_avg_storage = 0;
                    int v_counter = 0;

                    for(int i = 0; i < int(largest_v_values.size()); i++) {
                        v_avg_storage += largest_v_values[i];
                        v_counter++;
                    }

                    v_avg = int(v_avg_storage / float(v_counter));

                    cv::circle(rgb_image, cv::Point(int(resolution[0] / 2), v_avg), 5, cv::Scalar(0, 0, 255), -1);

                    if(v_avg >= 0) {
                        harvestZoneDetected = true;
                    }
                } else {
                    v_avg = -1;
                }
                // // if(abs(int(avgCupV / totalCupPixels) - targetCupV) <= cupVThreshold && abs(totalCupPixels - targetCupPointCount) <= cupPixelThreshold) {
                // if(abs(int(avgCupV / totalCupPixels) - targetCupV) <= cupVThreshold && abs(totalCupPixels) <= targetCupPointCount) {
                // // if(abs(int(avgCupV / totalCupPixels) - targetCupV) <= cupVThreshold) {
                //     harvestZoneDetected = true;
                //     // cout << "harvest zone detected" << endl;
                //     // cout << "blue detected! and initialCenteringDone: " << initialCenteringDone << endl;
                // } else {
                //     harvestZoneDetected = false;
                //     // cout << "harvest zone NOT detected" << endl;
                //     // cout << "blue NOT detected" << endl;
                // }
            }

            if(!harvesting && liftingY) {
                if(totalCupPixels > blueThresholdPixels) {
                    blueDetected = true;
                    // cout << "blue detected" << endl;
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

                    bool hue_condition = 0 <= hsv_pixel[0] && 180 >= hsv_pixel[0] && (30 > hsv_pixel[0] || 90 < hsv_pixel[0]);
                    bool sat_condition = 0 <= hsv_pixel[1] && 100 >= hsv_pixel[1];
                    bool val_condition = 0 <= hsv_pixel[2] && 255 >= hsv_pixel[2];

                    if (val < 0.2 && hue_condition && sat_condition && val_condition && val != 0 && croppingMask.at<unsigned char>(v, u) == 255) {
                        vineMask.at<unsigned char>(v, u) = 255;
                    }

                }
            }

            // cv::imshow("vinemask premorph", vineMask);

            // // Removing cups from vineMask to improve vine masking (as the gray color on the vines is a blue-based hue)
            cv::subtract(vineMask, cupMask, vineMask);

            cv::Mat bigKernel = cv::Mat::ones(7, 7, CV_8U);

            // More morphology to clean vineMask of noise
            cv::morphologyEx(vineMask, vineMask, cv::MORPH_OPEN, bigKernel);

            // cv::imshow("vineMask prefilter", vineMask);


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

            cv::Point minLoc(min_z_u, min_z_v);

            goalZ = minVal * 1000;

            // cout << "goalZ: " << goalZ << endl;


            int avg_u = int((max_u + min_u) / 2);

            xOffset = int(resolution[0] / 2) - avg_u;

            // cout << "xOffset: " << xOffset << endl;

            cv::Point centerX(avg_u, int(resolution[1] / 2));

            // cv::circle(rgb_image, minLoc, 5, cv::Scalar(0, 255, 0), -1);
            // cv::circle(rgb_image, centerX, 5, cv::Scalar(0, 0, 255), -1);

            // cv::imshow("image", rgb_image);

            char c = (char)cv::waitKey(1);

            if(c == 'q') {
                running = false;
            }

            counter++;
        }

        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "Exception in image_processing: " << e.what() << std::endl;
    }
}

void visual_servoing(EndEffectorConfig* mechanismPtr) {
    EndEffectorConfig& mechanism = *mechanismPtr;
    cout << "motors running" << endl;
    int xServoingSpeed = -1;
    int zServoingSpeed = -1;
    float zOffset = 0;
    int centeredCounter = -1;
    // bool x_centered = false;
    // bool z_centered = false;

    while (running) {

        if(goalZ >= 0 && !harvesting) { // initialized to -1, this check is to ensure image_processing gives valid goalZ before movement
            //cout << "driveStates (l, r): " << to_string(mechanism.leftMotor.driveState) << ", " << to_string(mechanism.rightMotor.driveState) << endl;
            // cout << "servoing" << endl;
            if(!findingZ && !blueDetected) {
                if(abs(xOffset) < x_deadband_buffer) {
                    xServoingSpeed = 0;
                    findingZ = true;
                    // x_centered = true;
                } else {
                    xServoingSpeed = xOffset;
                    // x_centered = false;
                }

                // cout << "servoing x at speed: " << xServoingSpeed << endl;

                mechanism.moveInX(-xServoingSpeed);
                mechanism.updateCurrentPosition();
            }

            if (findingZ && !blueDetected) { 
                zOffset = goalZ - hardware_buffer;
                if(abs(zOffset) < z_deadband_buffer) {
                    zServoingSpeed = 0;
                    findingZ = false;
                    // z_centered = true;
                } else {
                    zServoingSpeed = zOffset;
                    // z_centered = false;
                }

                // cout << "servoing z at speed: " << zServoingSpeed << endl;

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
    Cutter cutter(16);

    int harvestCount = 0;

    int curLimit = 30;
    int goalCur = 200;

    //Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent, min angle, max angle)
    MotorXM430 servo1(1, 5, curLimit, goalCur, 225, 315);
    MotorXM430 servo2(2, 5, curLimit, goalCur, 315, 225);

    
    EndEffectorConfig mechanism(0, 0); // defines the positioning mechanism


    //Accessing the motors function
    servo1.PrintOperatingMode();
    servo2.PrintOperatingMode();

    servo1.SetProfile(32000, 400); // have to be in velocity mode I think
    servo2.SetProfile(32000, 400); // have to be in velocity mode I think

    // Open grippers at startup
    drop(servo1, servo2);

    if(runMotors) {

        mechanism.calibrateZero(100);
        mechanism.updateCurrentPosition();

        cout << "current Z: " << mechanism.zPosition << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cout << "Moving in Z a little to make room in X" << endl;


        mechanism.goToPosition(0, 200, 150);
        mechanism.updateCurrentPosition();
    }

    cout << "Ready to Harvest" << endl;
    
    std::thread image_processing_thread(image_processing);
    std::thread motor_thread;

    if(runMotors) {
        std::thread motor_thread(visual_servoing, &mechanism);

        mechanism.yMotor.setSpeed(50);
        mechanism.yMotor.setAcceleration(10);
    

        while(running) {
            if(initialCenteringDone && !harvestZoneDetected) {
                liftingY = true;
                mechanism.yMotor.motorDriveY();
                mechanism.updateCurrentPosition();
            } else if (harvestZoneDetected) {
                harvesting = true;
                //move up in y set amount
                float currentY = mechanism.yPosition;
                int yDistanceToHarvestingZone;
                if(harvestCount == 0) {
                    yDistanceToHarvestingZone = 950;
                } else {
                    yDistanceToHarvestingZone = 1130;
                }

                cout << "moving up in y to get to top of cup" << endl;

                cout << "yDistanceToHarvestingZone: " << yDistanceToHarvestingZone << endl;

                float bottomPixel = v_avg;

                while(abs(mechanism.yPosition - currentY) <= (yDistanceToHarvestingZone - bottomPixel)) {
                    mechanism.yMotor.motorDriveY();
                    mechanism.updateCurrentPosition();
                    // cout << "mechanism.y: " << mechanism.yPosition << ", currentY: " << currentY << ", y distance travelled: " << abs(mechanism.yPosition - currentY) << endl;
                }

                cout << "y movement done, at top of cup" << endl;

                
                cout << "gripping and cutting" << endl;

                grip(servo1, servo2);
                usleep(500000);

                cutter.cutPlant();

                // usleep(500000);
                float currentX = mechanism.xPosition;
                currentY = mechanism.yPosition;
                float currentZ = mechanism.zPosition;

                cout << "currentPosition: X: " << mechanism.xPosition << ", Z: " << mechanism.zPosition << endl;


                mechanism.goToPosition(0, 150, 100);
                mechanism.updateCurrentPosition();
                cout << "dropping" << endl;

                drop(servo1, servo2);

                usleep(250000);

                cout << "going back to harvest zone" << endl;
                mechanism.goToPosition(currentX, currentZ, 150);

                cout << "raising up" << endl;

                while(abs(mechanism.yPosition - currentY) <= 650) {
                    // cout << "yPos: " << abs(mechanism.yPosition - currentY) << endl;
                    mechanism.yMotor.motorDriveY();
                    mechanism.updateCurrentPosition();
                }

                cout << "done raising up" << endl;

                // usleep(500000);

                harvestZoneDetected = false;
                harvesting = false;
                harvestCount++;

                if(harvestCount == 3) {
                    break;
                }

            }
        }
    }
    
    image_processing_thread.join();

    if(runMotors) {
        motor_thread.join();
    }
    


    return 0;
}
