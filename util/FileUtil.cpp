//
//  FileUtil.cpp
//  GestureTracking
//
//  Created by Alan Santos on 11/10/15.
//  Copyright (c) 2015 Alan Santos. All rights reserved.
//

#include "FileUtil.h"

//---------------------------------------------------------------------------
// Statics
//---------------------------------------------------------------------------
FileUtil* FileUtil::m_Instance = NULL;

FileUtil::FileUtil() {}

FileUtil::~FileUtil(){}

FileUtil&
FileUtil::getInstance() {
    if(m_Instance == NULL) {
        return *(m_Instance = new FileUtil());
    }
    return *m_Instance;
}

std::vector<std::string>&
FileUtil::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) elems.push_back(item);
    }
    return elems;
}

Point3D
FileUtil::getPointFile(const std::string str){
    std::vector<std::string> coordinates;
    Point3D newPoint;
    split(str, ' ', coordinates);
    newPoint.X = atof(coordinates[0].c_str());
    newPoint.Y = atof(coordinates[1].c_str());
    newPoint.Z = atof(coordinates[2].c_str());
    return newPoint;
}

Point3D
FileUtil::getPointFile(const std::string str, const int i){
    std::vector<std::string> coordinates;
    Point3D newPoint;
    split(str, ' ', coordinates);
    newPoint.X = atof(coordinates[i].c_str());
    newPoint.Y = atof(coordinates[i + 1].c_str());
    newPoint.Z = atof(coordinates[i + 2].c_str());
    return newPoint;
}

void
FileUtil::loadGestures(std::string nameFile){
    int start_s = clock();

    std::ifstream file;
    std::string row;
    std::vector<std::string> rows;

    mGesturesOneHand.clear();
    mGesturesTwoHands.clear();

    file.open(nameFile.c_str());

    if(file.is_open()){
        while (std::getline(file, row)){
            if(row.compare("end") == 0){
                extractGesture(rows);
                rows.clear();
            } else {
                rows.push_back(row);
            }
        }
    }

    TIME_METHOD_EXEC("loadGestures", start_s, clock());
    PRINT("Total Templates - " << mGesturesTwoHands.size() + mGesturesOneHand.size());
}

void
FileUtil::extractGesture(std::vector<std::string> rows){

    if(rows.empty()) return;

    std::vector<std::string> tokens;
    type_gesture newGesture;
    split(rows[0],' ', tokens);
    newGesture.name = tokens[1];
    newGesture.numHands = atoi(tokens[3].c_str());

    for (int i = 1; i < rows.size(); i++) {
        newGesture.handOne.positions.push_back(getPointFile(rows[i]));
        newGesture.handTwo.positions.push_back(getPointFile(rows[i], 3));
    }

    if(newGesture.handOne.positions.size() > 0 &&
        newGesture.handTwo.positions.size() > 0) {
        if(newGesture.numHands == 1){
            mGesturesOneHand.push_back(newGesture);
        } else {
            mGesturesTwoHands.push_back(newGesture);
        }
    }
}

void
FileUtil::clearHandGestures(){
    mGesturesOneHand.clear();
    mGesturesTwoHands.clear();
}
