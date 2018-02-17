#include "Util.h"

Util::Util(){
	m_CurvThreshold = 0.005;
	m_DougThreshold = 0.005;
}

Util::~Util(){}

void
Util::applyUniformByArcLength(type_gesture *gesture, double factor){
	MathUtil::uniformCurveByArcLength(&gesture->handOne.positions, factor);
	MathUtil::uniformCurveByArcLength(&gesture->handTwo.positions, factor);
}

void
Util::applyUniformByArcLength(std::vector<type_gesture> *gestures, double factor){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++){
		applyUniformByArcLength(&gestures->at(i), factor);
	}
}

void
Util::applyNormalization(type_gesture* gesture){
	Point3D min, max;
	max = MathUtil::findMaxFromTwo(gesture->handOne.positions, gesture->handTwo.positions);
	min = MathUtil::findMinFromTwo(gesture->handOne.positions, gesture->handTwo.positions);
	gesture->handOne.positions = MathUtil::normalizeTrajectory(gesture->handOne.positions, min, max);
	gesture->handTwo.positions = MathUtil::normalizeTrajectory(gesture->handTwo.positions, min, max);
}

void
Util::applyNormalization(std::vector<type_gesture>* gestures){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++){
		applyNormalization(&gestures->at(i));
	}
}

void
Util::applyLaplacian(type_gesture* gesture){
	gesture->handOne.positions = MathUtil::smoothMeanNeighboring(gesture->handOne.positions);
	gesture->handTwo.positions = MathUtil::smoothMeanNeighboring(gesture->handTwo.positions);
}

void
Util::applyLaplacian(std::vector<type_gesture>* gestures){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++) {
		applyLaplacian(&gestures->at(i));
	}
}

void
Util::applyBSpline(type_gesture* gesture){
	gesture->handOne.positions = BSpline::curvePoints(gesture->handOne.positions, NUM_STEP_BSPLINE);
	gesture->handTwo.positions = BSpline::curvePoints(gesture->handTwo.positions, NUM_STEP_BSPLINE);
}

void
Util::applyBSpline(std::vector<type_gesture>* gestures){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++) {
		applyBSpline(&gestures->at(i));
	}
}

void
Util::applyUniformBSpline(type_gesture* gesture){
	gesture->handOne.positions = BSpline::uniformFitting(gesture->handOne.positions);
	gesture->handTwo.positions = BSpline::uniformFitting(gesture->handTwo.positions);
}

void
Util::applyUniformBSpline(std::vector<type_gesture>* gestures){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++) {
		applyUniformBSpline(&gestures->at(i));
	}
}

void
Util::applyCurvature(type_gesture* gesture){
	gesture->handOne.positions = MathUtil::reduceByCurvature(gesture->handOne.positions, m_CurvThreshold);
	gesture->handTwo.positions = MathUtil::reduceByCurvature(gesture->handTwo.positions, m_CurvThreshold);
}

void
Util::applyCurvature(std::vector<type_gesture>* gestures){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++) {
		applyCurvature(&gestures->at(i));
	}
}

void
Util::applyDouglasPeucker(type_gesture* gesture){
	gesture->handOne.positions = MathUtil::simplify(gesture->handOne.positions, m_DougThreshold, false);
	gesture->handTwo.positions = MathUtil::simplify(gesture->handTwo.positions, m_DougThreshold, false);
}

void
Util::applyDouglasPeucker(std::vector<type_gesture>* gestures){
	size_t n = gestures->size();
	for (int i = 0; i < n; i++) {
		applyDouglasPeucker(&gestures->at(i));
	}
}

void
Util::generateGestureEqualSize(std::vector<type_gesture> *gestures){
	int n = gestures->size(), i = 0, j = 0, mean = 0, diff = 0;
	while(j < n){
		j = i + 1;
		while((j < n) && (gestures->at(i).name.compare(gestures->at(j).name) == 0)) j++;
		mean = getMeanPoints(*gestures, i, j - 1);
		for(int k = i; k < j; k++){
			diff = mean - gestures->at(k).handOne.positions.size();
			if(diff > 0){
				MathUtil::insertPoints(&gestures->at(k).handOne.positions, diff);
				MathUtil::insertPoints(&gestures->at(k).handTwo.positions, diff);
			} else if(diff < 0) {
				MathUtil::removePoints(&gestures->at(k).handOne.positions, diff);
				MathUtil::removePoints(&gestures->at(k).handTwo.positions, diff);
			}
		}
		i = j;
	}
}

void
Util::equalizeDatasetFromMax(std::vector<type_gesture> *gestures, std::string name){
	int n = gestures->size(), max, diff = 0;
	max = getMaxPoints(*gestures, 0, n - 1);
	for (int i = 0; i < n; i++){
		diff = max - gestures->at(i).handOne.positions.size();
		if (diff > 0){
			MathUtil::insertPoints(&gestures->at(i).handOne.positions, diff);
			MathUtil::insertPoints(&gestures->at(i).handTwo.positions, diff);
		} else if (diff < 0) {
			MathUtil::removePoints(&gestures->at(i).handOne.positions, diff);
			MathUtil::removePoints(&gestures->at(i).handTwo.positions, diff);
		}
	}
}

void
Util::equalizeDatasetFromMin(std::vector<type_gesture> *gestures, std::string name){
	int n = gestures->size(), min = 0, diff = 0;
	min = getMinPoints(*gestures, 0, n - 1);
	for (int i = 0; i < n; i++){
		diff = min - gestures->at(i).handOne.positions.size();
		if (diff > 0){
			MathUtil::insertPoints(&gestures->at(i).handOne.positions, diff);
			MathUtil::insertPoints(&gestures->at(i).handTwo.positions, diff);
		} else if(diff < 0) {
			MathUtil::removePoints(&gestures->at(i).handOne.positions, diff);
			MathUtil::removePoints(&gestures->at(i).handTwo.positions, diff);
		}
	}
}

void
Util::equalizeDatasetFromMean(std::vector<type_gesture> *gestures, std::string name){
	int n = gestures->size(), min = 0, diff = 0;
	min = getMeanPoints(*gestures, 0, n - 1);
	for (int i = 0; i < n; i++){
		diff = min - gestures->at(i).handOne.positions.size();
		if (diff > 0){
			MathUtil::insertPoints(&gestures->at(i).handOne.positions, diff);
			MathUtil::insertPoints(&gestures->at(i).handTwo.positions, diff);
		} else if (diff < 0) {
			MathUtil::removePoints(&gestures->at(i).handOne.positions, diff);
			MathUtil::removePoints(&gestures->at(i).handTwo.positions, diff);
		}
	}
}

int
Util::getMinPoints(vector<type_gesture> gestures, int i, int j) {
    int min = 99999999, n;
    for( ; i < j; i++) {
    	n = gestures[i].handOne.positions.size();
      if(min > n){
      	min = n;
      }
    }
    return min;
}

int
Util::getMaxPoints(vector<type_gesture> gestures, int i, int j) {
    int max = 0, n;
    for( ; i < j; i++) {
    	n = gestures[i].handOne.positions.size();
      if(max < n){
      	max = n;
      }
    }
    return max;
}

int
Util::getMeanPoints(vector<type_gesture> gestures, int i, int j) {
    int max = 0, min = 99999999, n;
    for( ; i < j; i++){
    	n = gestures[i].handOne.positions.size();
      if(max < n){
      	max = n;
      }
      if(min > n){
      	min = n;
      }
    }
    return (max + min) / 2;
}

void
Util::printGesture(const type_gesture gesture){
	for (int i = 0; i < gesture.handOne.positions.size(); i++){
		PRINT(gesture.handOne.positions[i].X << " " << gesture.handOne.positions[i].Y <<" "<<gesture.handOne.positions[i].Z <<" "<<
		gesture.handTwo.positions[i].X << " " << gesture.handTwo.positions[i].Y <<" "<<gesture.handTwo.positions[i].Z);
	}
}

void
Util::normCenterOriginGesture(type_gesture *gesture){
	gesture->handOne.positions = MathUtil::normCenterOrigin(gesture->handOne.positions);
	gesture->handTwo.positions = MathUtil::normCenterOrigin(gesture->handTwo.positions);
}
