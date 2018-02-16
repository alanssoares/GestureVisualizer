//
//  FileUtil.h
//  GestureTracking
//
//  Created by Alan Santos on 11/10/15.
//  Copyright (c) 2015 Alan Santos. All rights reserved.
//

#ifndef __GestureTracking__FileUtil__
#define __GestureTracking__FileUtil__

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ConstantsUtil.h"
#include "MathUtil.h"

class FileUtil {

private:

    FileUtil();
    ~FileUtil();

public:

    /**
     Break a string in substrings accordir with delimiter character
     @param std::string
     @param char
     @param std::vector<std::string>
     @return std::vector<std::string>
    */
    std::vector<std::string>& split(const std::string &s, char delim, std::vector<std::string> &elems);

    /**
     Clear gestures one and two hands
    */
    void clearHandGestures();

    /**
     Load the gestures from file
     @param std::string name file
    */
    void loadGestures(std::string nameFile);

    /**
     Extrac the gestures that was readed from the file and that was stored in a vector
     of strings
     @param std::vector<std::string>
    */
    void extractGesture(std::vector<std::string> rows);

    /**
     Return the Point3D from a std::string with the x,y,z positions
     @param std::string
     @return Point3D
    */
    Point3D getPointFile(std::string str);

    /**
     Return the Point3D from a std::string with the x,y,z positions
     @param std::string
     @param int column initial
     @return Point3D
    */
    Point3D getPointFile(const std::string str, const int i);

    /**
     Return the template gestures of one hand
     @return std::vector<type_gesture>
    */
    std::vector<type_gesture> getGesturesOneHand() { return mGesturesOneHand; };

    /**
     Return the template gestures of two hands
     @return std::vector<type_gesture>
    */
    std::vector<type_gesture> getGesturesTwoHands() { return mGesturesTwoHands; };

public:

    static FileUtil& getInstance();
    static FileUtil* m_Instance;

    std::vector<type_gesture>   mGesturesOneHand;
    std::vector<type_gesture>   mGesturesTwoHands;
};
#endif /* defined(__GestureTracking__FileUtil__) */
