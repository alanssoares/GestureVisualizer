//
//  ConstantsUtil.h
//  GestureVisualizer
//
//  Created by Alan Santos on 22/10/15.
//  Copyright (c) 2015 Alan Santos. All rights reserved.
//  Link - http://www.nsl.tuis.ac.jp/doxygen/JunkBox_Lib++/OpenNiDevice_8cpp_source.html
//

#ifndef GestureVisualizer_ConstantsUtil_h
#define GestureVisualizer_ConstantsUtil_h

#include <iostream>
#include <vector>

using namespace std;

typedef struct Point3D {
    double X;
    double Y;
    double Z;
    double curvature;
    double r;
    double g;
    double b;
} Point3D;

// Enum colors opengl
enum COLOR {
    GRAY = 0,
    BLUE = 1,
    GREEN = 2,
    YELLOW = 3,
    ORANGE = 4,
    PINK = 5,
    WHITE = 10
};

/* ------ Types methods for smooth */
enum METHOD_SMOOTH {
    MEAN_NEIGHBORING = 1,
    CUBIC_B_SPLINE = 2,
    CUBIC_BEZIER = 3
};
/* ---------- STATE Gesture ----------- */
enum STATE_GESTURE {
    GESTURE_DOING = 1,
    GESTURE_STOPED = 2
};

/* Object Types created to define an hand and an type of gesture */
typedef struct type_hand {
    int id_hand;
    int side_hand;
    vector<Point3D> positions;
    Point3D centroid;
} type_hand;

typedef struct type_gesture {
    int numHands;
    std::string name;
    type_hand handOne;
    type_hand handTwo;
} type_gesture;

typedef struct FeatureDescriptor {

    std::string name; // name of the gesture

    double mean; // length of the mean
    double centroidDerivative;
    double centroidLength; // the centroid length of the gesture
    double sumVariance; // sum of the variance
    double sumCurvature; // sum of the curvatures
    double sumOrientation; // orientation feature
    double standardDeviation; // the standard deviation
    std::vector<double> lc;
    std::vector<double> lsc;
    std::vector<double> velocity;
    std::vector<double> angles;
    std::vector<double> curvature;

} FeatureDescriptor;

/* Check the result and return 1 if XN_STATUS is not ok */
#define CHECK_RC(rc, message){ if(rc != XN_STATUS_OK){ printf(message, xnGetStatusString(rc)); return 1; }}
/* Check the result and return if XN_STATUS is not ok */
#define CHECK_RC_VOID(rc, message){ if(rc != XN_STATUS_OK){ printf(message, xnGetStatusString(rc)); return; }}
/* Print the message in console */
#define PRINT(message) { cout<<message<<endl; }
/* Print the time needed to execute the method */
#define TIME_METHOD_EXEC(method, start, stop) {PRINT("Method: "<<method<<", Time Execution: " << (stop-start)/double(CLOCKS_PER_SEC)*1000);}
/* Receive a time to sleep */
#define Sleep(t) usleep((t)*1000)
/* Define the minimum number of points of the motion doing */
#define NUM_LAST_POINTS 6.0
/* Define the min diff to verify if the hand is stopped or moving */
#define MIN_DIFF_LENGTH 3.0
/* The gesture is considered when have more then 3 points */
#define MIN_CONTROL_POINTS 3
/* Define the type of method that will be used to smooth the path */
#define TYPE_SMOOTH MEAN_NEIGHBORING
/* Is not an hand */
#define NOT_HAND -1
/* Define the threshold to remove points that there isn't large curvature */
#define THRESHOLD_CURVATURE 0.0005
/* ========= Mean Neighboring ======== */
#define NUMBER_SMOOTH_NB 1
/* ========= Cubic Spline ======== */
#define INCREMENT_RATE_INTERPOLATION 0.5
#define NUM_STEP_BSPLINE 1 //the quantity of points for each two interval points
/* ========= Cubic Spline ======== */
/* ========= Douglas-Peucker Radial ========= */
#define DPR_FACTOR_SIMPLIFY 0.01 //Factor for simplify
#define DPR_HIGH_QUALITY false //Flag for high quality for simplify
/* ========= End Douglas-Peucker Radial ========= */
#define GESTURE "gesture"
#define NAME_FILE_DATA "../datasets/msr_3d_action_raw.txt"
#define NAME_FILE_DATA_NORMALIZED "../datasets/msr_3d_action_origin_clean_spline_025.txt"
#define NAME_WINDOW_GR "GestureVisualizer"
#define GAP_WINDOW      25
#define GL_WIN_SIZE_X   1024 // 512 to gesture record
#define GL_WIN_SIZE_Y   512
#define GL_WIN_SIZE_MAIN_X 512
#define GL_WIN_SIZE_MAIN_Y 512

#endif
