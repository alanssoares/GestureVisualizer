#ifndef BSPLINE_H
#define BSPLINE_H

#include <string>
#include <cmath>
#include <vector>
#include "ConstantsUtil.h"

#define DELTA_T     1.0  /* time step factor for drawing each curve */

class BSpline {

public:
  	 BSpline(){};
    ~BSpline(){};

    /*
     Calc the basis function for a cubic B spline
     @param i
     @param t
    */
    static double baseFunction(int i, double t);

    /*
    Create the cubic bspline using the points and step
    @param points of control
    @param steps
    */
    static std::vector<Point3D> curvePoints(std::vector<Point3D> points, int steps);

    /*
    Evaluate an ith point on the B spline
    @param i
    @param t
    @param points
    */
    static Point3D predictPoint(int i, double t, std::vector<Point3D> points);

    /*
      This function evaluates the uniform cubic B-spline.
    */
    static double baseFunctionUniform(double t);

    /* This function approximates the data with spline curves.
      https://www.it.uu.se/edu/course/homepage/grafik1/ht07/examples/curves.cpp
    */
    static std::vector<Point3D> uniformFitting(std::vector<Point3D> points);

};

#endif // BSPLINE_H
