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

#include <EndEffectorConfig.h>
#include <MotorConfig.h>
//#include <XM430.h>
//#include <Cutter.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

std::atomic<bool> running(true);
std::atomic<int> goalZ(0);
double hardware_buffer = 50;
double deadband_buffer = 5;

int resolution[2] = {424, 240};

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

        rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
        rs2::spatial_filter spat_filter;    // Spatial - edge-preserving spatial smoothing
        rs2::temporal_filter temp_filter;

        while (running) {
            // Wait for the next set of frames from the camera
            rs2::frameset frames = pipe.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame().as<rs2::depth_frame>();

            depth_frame = spat_filter.process(depth_frame);
            depth_frame = temp_filter.process(depth_frame);

            // Convert RealSense frame to OpenCV matrix
            cv::Mat rgb_image(cv::Size(resolution[0], resolution[1]), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat hsv_image;
            cv::Mat depth_image(cv::Size(resolution[0], resolution[1]), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            cv::cvtColor(rgb_image, hsv_image, COLOR_BGR2HSV, 0);

            cv::Mat depth_visual;
            depth_image.convertTo(depth_visual, CV_8U, 255.0 / 1000);

            cv::imshow("depth_visual", depth_visual);

            // Creating Mask using HSV thresholding
            cv::Mat cupMask(cv::Size(resolution[0], resolution[1]), CV_8UC1);
            cv::Scalar lowerHSV(100, 200, 0);
            cv::Scalar upperHSV(150, 255, 255);

            cv::inRange(hsv_image, lowerHSV, upperHSV, cupMask);

            // Morphology to clean image
            cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
            cv::morphologyEx(cupMask, cupMask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(cupMask, cupMask, cv::MORPH_OPEN, kernel);

            cv::imshow("mask", cupMask);

            cv::Mat vineMask = cv::Mat::zeros(cv::Size(resolution[0], resolution[1]), CV_8UC1);

            // Depth sensing across a cropped ROI to find nonzero minZ as well as to isolate vine geometry for masking
            // rows is resolution[1], cols is resolution[0]
            double minVal = std::numeric_limits<double>::max();
            cv::Point minLoc(0, 0);

            for (int v = 0; v < depth_image.rows - 60; v++) {
                for (int u = 70; u < depth_image.cols - 70; u++) {
                    double val = depth_frame.get_distance(u, v);
                    cv::Vec3b hsv_pixel = hsv_image.at<cv::Vec3b>(v, u);

                    bool hue_condition = 100 <= hsv_pixel[0] && 150 >= hsv_pixel[0];
                    bool sat_condition = 0 <= hsv_pixel[1] && 150 >= hsv_pixel[1];
                    bool val_condition = 0 <= hsv_pixel[2] && 255 >= hsv_pixel[2];

                    if (val < 0.3 && hue_condition && sat_condition && val_condition) {
                        vineMask.at<unsigned char>(v, u) = 255;
                        if (val != 0 && val < minVal && val > 0) {
                            minVal = val;
                            minLoc = cv::Point(u, v);
                        }
                    }
                }
            }

            goalZ = minVal * 1000;

            // Removing cups from vineMask to prepare for vine color segmentation
            cv::subtract(vineMask, cupMask, vineMask);

            cv::imshow("vineMask before morph", vineMask);

            cv::Mat kernel_large = cv::Mat::ones(7, 7, CV_8U);
            cv::morphologyEx(vineMask, vineMask, cv::MORPH_OPEN, kernel_large);

            cv::imshow("vineMask", vineMask);

            double avg_u = 0;
            int u_count = 0;

            for (int v = 0; v < vineMask.rows; v++) {
                for (int u = 0; u < vineMask.cols; u++) {
                    if (vineMask.at<unsigned char>(v, u) != 0) {
                        avg_u = avg_u + u;
                        u_count++;
                    }
                }
            }

            cv::Mat vineMaskedImage;
            cv::bitwise_and(rgb_image, rgb_image, vineMaskedImage, vineMask);

            cv::imshow("vine masked rgb_image", vineMaskedImage);

            avg_u = int(avg_u / u_count);

            cv::Point centerX(avg_u, int(resolution[1] / 2));

            cv::circle(rgb_image, minLoc, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(rgb_image, centerX, 5, cv::Scalar(0, 0, 255), -1);

            cv::imshow("min z point and avg x", rgb_image);

            cv::waitKey(1);
        }

        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "Exception in image_processing: " << e.what() << std::endl;
    }
}

void drive_motors(EndEffectorConfig mechanism) {

    cout << "motors running" << endl;
    try {
        while (running) {
            if (goalZ > 0) { // initialized to 0, this check is to ensure image_processing gives valid goalZ before movement
                mechanism.moveInZ(goalZ - hardware_buffer);
                cout << "goal Z: " << goalZ << " hardware_buffer: " << hardware_buffer << endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception in drive_motors: " << e.what() << std::endl;
    }
}

// main function
int main() {   
    // INITIALIZE ALL MOTORS AND OBJECTS//
    EndEffectorConfig mechanism(0, 0); // defines the positioning mechanism

    // UPDATE Y STAGE PINS
    // MotorConfig yStage(-1, -1, -1, -1); // defines the y stage motor, will need to determine if up/down is positive or negative speed

    mechanism.calibrateZero(100);
    mechanism.updateCurrentPosition();

    cout << "current Z: " << mechanism.zPosition << endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    cout << "Moving in Z a little to make room in X" << endl;

    mechanism.goToPosition(0, 150, 50);
    mechanism.updateCurrentPosition();

    cout << "current Z: " << mechanism.zPosition << endl;
    cout << "Ready to Harvest" << endl;
    
    std::thread image_processing_thread(image_processing);
    std::thread motor_thread(drive_motors, mechanism);

    image_processing_thread.join();
    motor_thread.join();

    return 0;
}
