#ifndef CUTTER_H
#define CUTTER_H


#include <stdio.h> 
#include <stdlib.h>
#include <wiringPi.h>   // Include WiringPi library!
#include <time.h>       // For NANOS function
#include <iostream>
#include <cmath>        // For sqrt and other math functions
#include <atomic>

// #include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <cmath>


class Cutter
{

    public:
        uint8_t cutPin; 
        uint8_t maxCutAttempts;

        Cutter(uint8_t pin1);

        void setAttempts(uint8_t newAttempts);
        void cutPlant();
        void resetCut();
        double distance(float* pt1, float* pt2);
        // double mean(std::vector<int>& v);
        bool fiducials();
};

#endif // CUTTER_H