#ifndef MYTEST_H
#define MYTEST_H

#include <chrono>
#include <cstdio>
#include <iostream>
//#include "utility.hpp"
#include "depthai/depthai.hpp"  // Includes common necessary includes for development using depthai library


// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> syncNN{true};


void camera(void);


#endif // MYTEST_H
