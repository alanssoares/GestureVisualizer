#ifndef UTIL_H__
#define UTIL_H__

#include "ConstantsUtil.h"
#include "MathUtil.h"
#include "FileUtil.h"

class Util {

public:
	Util();
	~Util();

	//Print all informations of the gesture
	void printGesture(const type_gesture gesture);
	// Get the min of points from a range of gestures
	int  getMinPoints(vector<type_gesture> gestures, int i, int j);
	// Get the max of points from a range of gestures
	int  getMaxPoints(vector<type_gesture> gestures, int i, int j);
	//Get the mean number of points of an set of gestures of the same type
	int  getMeanPoints(vector<type_gesture> gestures, int i, int j);
	//Generate gestures with equal number of points from mean
	void generateGestureEqualSize(std::vector<type_gesture> *gestures);
	//Equalize gestures with equal number of points from min
	void equalizeDatasetFromMin(std::vector<type_gesture> *gestures, std::string name);
	//Equalize gestures with equal number of points from max
	void equalizeDatasetFromMax(std::vector<type_gesture> *gestures, std::string name);
	//Equalize gestures with equal number of points from mean
	void equalizeDatasetFromMean(std::vector<type_gesture> *gestures, std::string name);
	//Generate a file with gestures centered in the origin using centroid
	void normCenterOriginGesture(type_gesture *gesture);
	//Apply uniform distance by arc length
	void applyUniformByArcLength(type_gesture *gesture, double factor);
	void applyUniformByArcLength(std::vector<type_gesture> *gestures, double factor);
	//Apply normalization in the interval [-1,1] in the gestures
	void applyNormalization(type_gesture *gesture);
	void applyNormalization(std::vector<type_gesture>* gestures);
	//Apply the method Laplacian in the gestures
	void applyLaplacian(type_gesture *gesture);
	void applyLaplacian(std::vector<type_gesture> *gestures);
	//Apply the method uniform B-Spline in the gestures
	void applyUniformBSpline(type_gesture *gesture);
	void applyUniformBSpline(std::vector<type_gesture> *gestures);
	//Apply the method B-Spline in the gestures
	void applyBSpline(type_gesture *gesture);
	void applyBSpline(std::vector<type_gesture> *gestures);
	//Apply the method Curvature for simplification in the gestures
	void applyCurvature(type_gesture *gesture);
	void applyCurvature(std::vector<type_gesture> *gestures);
	//Apply the method DouglasPeucker for simplification in the gestures
	void applyDouglasPeucker(type_gesture *gesture);
	void applyDouglasPeucker(std::vector<type_gesture> *gestures);

	float m_CurvThreshold, m_DougThreshold;
};

#endif //UTIL_H__
