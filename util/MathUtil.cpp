//
//  MathUtil.cpp
//  GestureTracking
//
//  Created by Alan Santos on 22/10/15.
//  Copyright (c) 2015 Alan Santos. All rights reserved.
//

#include "MathUtil.h"

std::string
MathUtil::intToString(int n){
    std::ostringstream converter;
    converter << n;
    return converter.str();
}

std::string
MathUtil::floatToString(float n){
    std::ostringstream converter;
    converter << n;
    return converter.str();
}

double
MathUtil::length(Point3D point){
    return sqrt(pow(point.X, 2) + pow(point.Y, 2) + pow(point.Z, 2));
}

Point3D
MathUtil::subtract(Point3D a, Point3D b){
    a.X = b.X - a.X;
    a.Y = b.Y - a.Y;
    a.Z = b.Z - a.Z;
    return a;
}

Point3D
MathUtil::normalize(Point3D point) {
    float len = length(point);
    point.X = point.X / len;
    point.Y = point.Y / len;
    point.Z = point.Z / len;
    return point;
}

Point3D
MathUtil::sum(Point3D a, Point3D b) {
    Point3D c;
    c.X = a.X + b.X;
    c.Y = a.Y + b.Y;
    c.Z = a.Z + b.Z;
    return c;
}

Point3D
MathUtil::interpolate(Point3D p0, Point3D p1, float t){
    Point3D c;

    if(t > 1.f) t = 1.f;
    if(t < 0.f) t = 0.f;

    c.X = (1 - t) * p0.X + t * p1.X;
    c.Y = (1 - t) * p0.Y + t * p1.Y;
    c.Z = (1 - t) * p0.Z + t * p1.Z;

    return c;
}

double
MathUtil::getAngleBetween2Points(Point3D a, Point3D b){
    double mU, mV, mUV, uv, angle;

    //Produto vetorial
    uv = a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    //Módulo dos vetores
    mU = sqrt(pow(a.X, 2) + pow(a.Y, 2) + pow(a.Z, 2));
    mV = sqrt(pow(b.X, 2) + pow(b.Y, 2)) + pow(b.Z, 2);

    //Produto vetorial dos módulos
    mUV = mU * mV;

    angle = acos(cos(uv/mUV));

    return angle;
}

double
MathUtil::getMaxValue(vector<double> values){
    size_t n = values.size();
    double max = 0.0;
    for (int i = 0; i < n; i++) {
        if (max < values[i]) {
            max = values[i];
        }
    }

    return max;
}

Point3D
MathUtil::calcCentroid(vector<Point3D> positions){
    size_t n = positions.size();
    Point3D centroid;
    centroid.X = centroid.Y = centroid.Z = 0;
    for(int i = 0; i < n; i++) {
        centroid.X += positions[i].X;
        centroid.Y += positions[i].Y;
        centroid.Z += positions[i].Z;
    }
    //Sum 1 to prevent division by zero
    centroid.X = (centroid.X + 1)/(n + 1) - 1;
    centroid.Y = (centroid.Y + 1)/(n + 1) - 1;
    centroid.Z = (centroid.Z + 1)/(n + 1) - 1;

    return centroid;
}

vector<Point3D>
MathUtil::translateToOrigin(vector<Point3D> positions){
    size_t n = positions.size();
    Point3D centroid = calcCentroid(positions);

    for(int i = 0; i < n; i++) {
        positions[i].X -= centroid.X;
        positions[i].Y -= centroid.Y;
        positions[i].Z -= centroid.Z;
    }

    return positions;
}

vector<Point3D>
MathUtil::normalizeTrajectory(vector<Point3D> positions, Point3D min, Point3D max){
    size_t n = positions.size();
    for(int i = 0; i < n; i++) {
        positions[i].X = ((positions[i].X - min.X) / (max.X - min.X));
        positions[i].Y = ((positions[i].Y - min.Y) / (max.Y - min.Y));
        positions[i].Z = ((positions[i].Z - min.Z) / (max.Z - min.Z));
    }
    return positions;
}

Point3D
MathUtil::findMinFromTwo(vector<Point3D> a, vector<Point3D> b){
  Point3D min1 = MathUtil::minValueXYZ(a);
  Point3D min2 = MathUtil::minValueXYZ(b);
  if(min1.X > min2.X) min1.X = min2.X;
  if(min1.Y > min2.Y) min1.Y = min2.Y;
  if(min1.Z > min2.Z) min1.Z = min2.Z;
  return min1;
}

Point3D
MathUtil::findMaxFromTwo(vector<Point3D> a, vector<Point3D> b){
  Point3D max1 = MathUtil::maxValueXYZ(a);
  Point3D max2 = MathUtil::maxValueXYZ(b);
  if(max1.X < max2.X) max1.X = max2.X;
  if(max1.Y < max2.Y) max1.Y = max2.Y;
  if(max1.Z < max2.Z) max1.Z = max2.Z;
  return max1;
}

Point3D
MathUtil::minValueXYZ(vector<Point3D> positions){
    size_t n = positions.size();
    Point3D minPos;
    minPos.X = 99999999;
    minPos.Y = 99999999;
    minPos.Z = 99999999;
    for(int i = 0; i < n; i++) {
        if(positions[i].X < minPos.X) minPos.X = positions[i].X;
        if(positions[i].Y < minPos.Y) minPos.Y = positions[i].Y;
        if(positions[i].Z < minPos.Z) minPos.Z = positions[i].Z;
    }
    return minPos;
}

Point3D
MathUtil::maxValueXYZ(vector<Point3D> positions){
    size_t n = positions.size();
    Point3D maxPos;
    maxPos.X = -99999999;
    maxPos.Y = -99999999;
    maxPos.Z = -99999999;
    for(int i = 0; i < n; i++) {
        if(positions[i].X > maxPos.X) maxPos.X = positions[i].X;
        if(positions[i].Y > maxPos.Y) maxPos.Y = positions[i].Y;
        if(positions[i].Z > maxPos.Z) maxPos.Z = positions[i].Z;
    }
    return maxPos;
}

std::vector<Point3D>
MathUtil::smoothMeanNeighboring(std::vector<Point3D> positions){
    std::vector<Point3D> smoothed;
    Point3D meanPoint;
    const int numPoints = 3;
    size_t n  = positions.size();
    if(n >= MIN_CONTROL_POINTS){
        smoothed.push_back(positions.front());
        for (int i = 0; i < n - 2; i++) {
            meanPoint.X = (positions[i].X + positions[i + 1].X + positions[i + 2].X)/numPoints;
            meanPoint.Y = (positions[i].Y + positions[i + 1].Y + positions[i + 2].Y)/numPoints;
            meanPoint.Z = (positions[i].Z + positions[i + 1].Z + positions[i + 2].Z)/numPoints;
            smoothed.push_back(meanPoint);
        }
        smoothed.push_back(positions.back());
    } else {
        //Is not possible smooth, return the original positions
        smoothed = positions;
    }
    return smoothed;
}

std::vector<Point3D>
MathUtil::simplify(std::vector<Point3D> points, double tolerance, bool highestQuality){
    double sqTolerance = pow(tolerance, 2);

    if(points.size() <= 2) {
        return points;
    }

    if(highestQuality){
        points = simplifyRadialDist(points, sqTolerance);
    }

    return simplifyDouglasPeucker(points, sqTolerance);
}

std::vector<Point3D>
MathUtil::simplifyRadialDist(std::vector<Point3D> points, double sqTolerance){
    Point3D prevPoint, point;
    std::vector<Point3D> newPoints;
    double sqDistance = 0.0;
    size_t n = points.size();
    prevPoint = points[0];
    newPoints.push_back(prevPoint);

    for (int i = 1; i < n; i++) {
        point = points[i];
        sqDistance = getArcLength(point, prevPoint);
        if (sqDistance > sqTolerance) {
            newPoints.push_back(point);
            prevPoint = point;
        }
    }

    if (!pointsEqual(prevPoint, point)) {
        newPoints.push_back(point);
    }

    return newPoints;
}

std::vector<Point3D>
MathUtil::simplifyDouglasPeucker(std::vector<Point3D> points, double sqTolerance){
    std::vector<Point3D> simplified;
    simplified.push_back(points.front());
    simplifyDPStep(points, 0, points.size() - 1, sqTolerance, &simplified);
    simplified.push_back(points.back());
    return simplified;
}

void
MathUtil::simplifyDPStep(std::vector<Point3D> points, int first, int last, double sqTolerance, std::vector<Point3D> *simplified){
    double maxSqDist = sqTolerance;
    int index = -1;

    for (int i = first + 1; i < last; i++) {
        double sqDist = getDistancePointToSegment(points[i], points[first], points[last]);
        if (sqDist > maxSqDist) {
            index = i;
            maxSqDist = sqDist;
        }
    }

    if (maxSqDist > sqTolerance) {
        if (index - first > 1) {
            simplifyDPStep(points, first, index, sqTolerance, simplified);
        }

        simplified->push_back(points[index]);

        if (last - index > 1) {
            simplifyDPStep(points, index, last, sqTolerance, simplified);
        }
    }
}

std::vector<Point3D>
MathUtil::reduceByCurvature(std::vector<Point3D> points, double threshold){
    std::vector<Point3D> newPoints;
    size_t n = points.size();
    if(n < 3) return points;
    double curvature = 0.0;

    newPoints.push_back(points.front());

    for (int i = 0; i < n - 2; i+=2){
        curvature = calcCurvature(points[i], points[i + 1], points[i + 2]);
        if(curvature > threshold){
            newPoints.push_back(points[i + 1]);
        }
    }

    newPoints.push_back(points.back());

    return newPoints;
}

float
MathUtil::calcCurvature(Point3D a, Point3D b, Point3D c){
    Point3D l1, l2;
    l1 = subtract(a, b);
    l2 = subtract(b, c);
    return length(subtract(l1, l2));
}

double
MathUtil::calcSlope(Point3D p1, Point3D p2) {
    Point3D p3 = subtract(p1, p2);
    double run = sqrt(pow(p3.X,2) + pow(p3.Y,2));
    return p3.Z / run;
}

double
MathUtil::getArcLength(Point3D p1, Point3D p2){
    return length(subtract(p1, p2));
}

double
MathUtil::getDistancePointToSegment(Point3D p, Point3D p1, Point3D p2){
    double x = p1.X,
        y = p1.Y,
        z = p1.Z,
        dx = p2.X - x,
        dy = p2.Y - y,
        dz = p2.Z - z;

    if (dx != 0 || dy != 0 || dz != 0) {
        double t = ((p.X - x) * dx + (p.Y - y) * dy + (p.Z - z) * dz) / (pow(dx,2) + pow(dy,2) + pow(dz,2));
        if (t > 1.0) {
            x = p2.X;
            y = p2.Y;
            z = p2.Z;
        } else if (t > 0.0) {
            x += dx * t;
            y += dy * t;
            z += dz * t;
        }
    }

    dx = p.X - x;
    dy = p.Y - y;
    dz = p.Z - z;

    return pow(dx,2) + pow(dy,2) + pow(dz,2);
}

std::vector<Point3D>
MathUtil::smooth(std::vector<Point3D> trajectory){
    switch(TYPE_SMOOTH){
        case CUBIC_B_SPLINE:
            trajectory = BSpline::curvePoints(trajectory, NUM_STEP_BSPLINE);
            break;
        default:
            trajectory = smoothMeanNeighboring(trajectory);
            break;
    }
    return trajectory;
}

std::vector<Point3D>
MathUtil::normCenterOrigin(std::vector<Point3D> trajectory) {
    //Translate the hand trajectory to origin
    trajectory = translateToOrigin(trajectory);
    //Normalize between the interval -1 to 1
    return normalizeTrajectory(trajectory, minValueXYZ(trajectory), maxValueXYZ(trajectory));
}

std::vector<Point3D>
MathUtil::smoothAndReduce(std::vector<Point3D> trajectory) {
    //Remove points according with curvature
    trajectory = reduceByCurvature(trajectory, THRESHOLD_CURVATURE);
    //Smooth the trajectory
    return smooth(trajectory);
}

bool
MathUtil::pointsEqual(Point3D p1, Point3D p2){
    return p1.X == p2.X && p1.Y == p2.Y && p1.Z == p2.Z;
}

void
MathUtil::insertPoints(std::vector<Point3D> *points, int diff){
    int i, n, index = -1;
    double curv = 0.0, maxCurv = 0.0;
    Point3D newPoint;
    while(diff > 0){
        n = points->size();
        for (i = 0; i < n - 2; i+=2){
            curv = calcCurvature(points->at(i), points->at(i + 1), points->at(i + 2));
            if (curv > maxCurv){
                maxCurv = curv;
                index = i + 1;
            }
        }
        if(index > 0){
          if (index + 1 < n) {
            newPoint = interpolate(points->at(index - 1), points->at(index + 1), 0.5);
          } else {
            newPoint = interpolate(points->at(index - 1), points->at(index), 0.5);
          }
          points->insert(points->begin() + index, newPoint);
        }
        diff--;
        maxCurv = 0.0;
        index = -1;
    }
}

void
MathUtil::removePoints(std::vector<Point3D> *points, int diff){
    int i, n, index = -1;
    double curv = 0.0, minCurv = 9999999;
    while(diff < 0){
        n = points->size();
        for (i = 0; i < n - 2; i+=2){
            curv = calcCurvature(points->at(i), points->at(i + 1), points->at(i + 2));
            if (curv < minCurv){
                minCurv = curv;
                index = i + 1;
            }
        }
        if (index > 0){
          if (index + 1 < n) {
            points->erase(points->begin() + index);
          } else {
            points->erase(points->begin() + index - 1);
          }
        }
        diff++;
        minCurv = 99999999;
        index = -1;
    }
}

void
MathUtil::uniformCurveByArcLength(std::vector<Point3D> *points, double dL){
  size_t n = points->size();
  double aL, t = 1.0;
  int i = 0;
  Point3D prev;
  while(i < n - 1) {
    aL = getArcLength(points->at(i), points->at(i + 1));
    if(aL < dL){
      points->erase(points->begin() + i + 1);
      n--;
      continue;
    }
    while(aL > dL){
      prev = interpolate(points->at(i), points->at(i + 1), t);
      aL = getArcLength(points->at(i), prev);
      t -= 0.01;
    }
    points->at(i + 1) = prev;
    i++;
    t = 1.0;
  }
}
