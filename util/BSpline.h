#ifndef BSPLINE_H
#define BSPLINE_H

#include <string>
#include <cmath>
#include <vector>
#include "ConstantsUtil.h"

#define DELTA_T     1.0  /* time step factor for drawing each curve */

using namespace std;

class BSpline {

public:
  	 BSpline(){};
    ~BSpline(){};

    // -------- NEW IMPLEMENTATION ------
    // https://github.com/Tagussan/BSpline/blob/master/BSpline.js
    double seqAt(int i, int dim, int degree);
    double basisDeg2(double x);
    double basisDeg3(double x);
    double basisDeg4(double x);
    double basisDeg5(double x);
    double getInterpol(int degree, int dim, double t);
    Point3D calcAt(double t, double degree);
    void setPoints(std::vector<Point3D> points) { m_Points = points; }
    std::vector<Point3D> compute(std::vector<Point3D> points, int degree, double t);
    std::vector<Point3D> m_Points;
    // ---------------------------------------------------------------

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
