#include "BSpline.h"

double
BSpline::seqAt(int i, int dim, int degree) {
  int margin = degree + 1;

  if (i < margin) {
    if (dim == 0) return m_Points[0].X;
    if (dim == 1) return m_Points[0].Y;
    if (dim == 2) return m_Points[0].Z;
  } else if (m_Points.size() + margin <= i) {
    if (dim == 0) return m_Points[m_Points.size() - 1].X;
    if (dim == 1) return m_Points[m_Points.size() - 1].Y;
    if (dim == 2) return m_Points[m_Points.size() - 1].Z;
  }

  if (dim == 0) return m_Points[i - margin].X;
  if (dim == 1) return m_Points[i - margin].Y;
  if (dim == 2) return m_Points[i - margin].Z;

  return 0.0; // ERROR
}

double
BSpline::basisDeg2(double x) {
  if (-0.5 <= x && x < 0.5){
    return 0.75 - x * x;
  } else if(0.5 <= x && x <= 1.5){
    return 1.125 + (-1.5 + x/2.0) * x;
  } else if(-1.5 <= x && x < -0.5){
    return 1.125 + (1.5 + x/2.0) * x;
  } else{
    return 0;
  }
}

double
BSpline::basisDeg3(double x) {
  if (-1 <= x && x < 0) {
      return 2.0/3.0 + (-1.0 - x/2.0) * x * x;
  } else if (1 <= x && x <= 2) {
      return 4.0/3.0 + x*(-2.0 + (1.0 - x/6.0) * x);
  } else if (-2 <= x && x < -1) {
      return 4.0/3.0 + x*(2.0 + (1.0 + x/6.0) * x);
  } else if (0 <= x && x < 1) {
      return 2.0/3.0 + (-1.0 + x/2.0) * x * x;
  } else {
      return 0;
  }
}

double
BSpline::basisDeg4(double x) {
  if (-1.5 <= x && x < -0.5) {
      return 55.0/96.0 + x * (-(5.0/24.0) + x*(-(5.0/4.0) + (-(5.0/6.0) - x/6.0)*x));
  } else if (0.5 <= x && x < 1.5) {
      return 55.0/96.0 + x*(5.0/24.0 + x*(-(5.0/4.0) + (5.0/6.0 - x/6.0)*x));
  } else if (1.5 <= x && x <= 2.5) {
      return 625.0/384.0 + x*(-(125.0/48.0) + x*(25.0/16.0 + (-(5.0/12.0) + x/24.0)*x));
  } else if (-2.5 <= x && x <= -1.5) {
      return 625.0/384.0 + x*(125.0/48.0 + x*(25.0/16.0 + (5.0/12.0 + x/24.0)*x));
  } else if (-1.5 <= x && x < 1.5) {
      return 115.0/192.0 + x*x*(-(5.0/8.0) + x*x/4.0);
  } else {
      return 0;
  }
}

double
BSpline::basisDeg5(double x) {
  if (-2 <= x && x < -1){
      return 17.0/40.0 + x*(-(5.0/8.0) + x*(-(7.0/4.0) + x*(-(5.0/4.0) + (-(3.0/8.0) - x/24.0)*x)));
  } else if (0 <= x && x < 1){
      return 11.0/20.0 + x*x*(-(1.0/2.0) + (1.0/4.0 - x/12.0)*x*x);
  } else if (2 <= x && x <= 3){
      return 81.0/40.0 + x*(-(27.0/8.0) + x*(9.0/4.0 + x*(-(3.0/4.0) + (1.0/8.0 - x/120.0)*x)));
  } else if (-3 <= x && x < -2){
      return 81.0/40.0 + x*(27.0/8.0 + x*(9.0/4.0 + x*(3.0/4.0 + (1.0/8.0 + x/120.0)*x)));
  } else if (1 <= x && x < 2){
      return 17.0/40.0 + x*(5.0/8.0 + x*(-(7.0/4.0) + x*(5.0/4.0 + (-(3.0/8.0) + x/24.0)*x)));
  } else if (-1 <= x && x < 0){
      return 11.0/20.0 + x*x*(-(1.0/2.0) + (1.0/4.0 + x/12.0)*x*x);
  } else {
      return 0;
  }
}

double
BSpline::getInterpol(int degree, int dim, double t) {
  int tInt = floor(t);
  int rangeInt = 2;
  if (degree == 4 || degree == 5) {
    rangeInt = 3;
  }
  double result = 0;
  for (int i = tInt - rangeInt; i <= tInt + rangeInt;i++){
    if (degree == 2) {
      result += seqAt(i, dim, degree) * basisDeg2(t - i);
    } else if (degree == 3) {
      result += seqAt(i, dim, degree) * basisDeg3(t - i);
    } else if (degree == 4) {
      result += seqAt(i, dim, degree) * basisDeg4(t - i);
    } else if (degree == 5) {
      result += seqAt(i, dim, degree) * basisDeg5(t - i);
    }
  }
  return result;
}

Point3D
BSpline::calcAt(double t, double degree) {
  Point3D point;
  t = t * ((degree + 1) * 2 + m_Points.size());//t must be in [0,1]
  point.X = getInterpol(degree, 0, t);
  point.Y = getInterpol(degree, 1, t);
  point.Z = getInterpol(degree, 2, t);
  return point;
}

double
BSpline::baseFunction(int i, double t){
    switch (i) {
        case -2:
            return (((-t + 3) * t - 3) * t + 1) / 6;
        case -1:
            return (((3 * t - 6) * t) * t + 4) / 6;
        case 0:
            return (((-3 * t + 3) * t + 3) * t + 1) / 6;
        case 1:
            return (t * t * t) / 6;
    }
    return 0; //we only get here if an invalid i is specified
}

double
BSpline::baseFunctionUniform(double t){
   double tp2, tp1, tm2, tm1;

   if (t <= -2.0) return 0.0;

   if (t <= -1.0) {
      tp2 = t + 2.0;
      return tp2 * tp2 * tp2 / 6.0;
   } else if (t <= 0.0) {
      tp2 = t + 2.0;
      tp1 = t + 1.0;
      tp2 = tp2 * tp2 * tp2 / 6.0;
      tp1 = 2.0 * tp1 * tp1 * tp1 /3.0;
      return tp2 - tp1;
   } else if (t <= 1.0) {
      tm1 = 1.0 - t;
      tm2 = 2.0 - t;
      tm1 = 2.0 * tm1 * tm1 * tm1 / 3.0;
      tm2 = tm2 * tm2 * tm2 / 6.0;
      return tm2 - tm1;
   } else if (t <= 2.0) {
      tm2 = 2.0 - t;
      return tm2 * tm2 * tm2 / 6.0;
   }

   return 0.0;
}

std::vector<Point3D>
BSpline::curvePoints(std::vector<Point3D> points, int steps){
	 std::vector<Point3D> curve;
   int numPts = points.size() - 1;
   int pts = numPts * steps + 1;
   curve.push_back(predictPoint(2, 0, points));
   for (int i = 2; i < numPts; i++) {
     for (int j = 1; j <= steps; j++) {
       curve.push_back(predictPoint(i, j / (double) steps, points));
     }
   }
   return curve;
}

std::vector<Point3D>
BSpline::uniformFitting(std::vector<Point3D> points){
  size_t n = points.size();
  double bt1, bt2, bt3, bt4;
  std::vector<Point3D> curve;
  Point3D newPoint;

  /* Make the two endpoints multiple so that they are interpolated. */
  points.insert(points.begin(), points.front());
  points.insert(points.begin(), points.front());
  points.push_back(points.back());
  points.push_back(points.back());

  for (int i = 0; i < n; i++){
      for (double t = DELTA_T; t < 1.0 + DELTA_T / 2.0; t += DELTA_T){
        bt1 = baseFunctionUniform(t - 2.0);
        bt2 = baseFunctionUniform(t - 1.0);
        bt3 = baseFunctionUniform(t);
        bt4 = baseFunctionUniform(t + 1.0);
        newPoint.X = points[i].X*bt4 + points[i+1].X*bt3 + points[i+2].X*bt2 + points[i+3].X*bt1;
        newPoint.Y = points[i].Y*bt4 + points[i+1].Y*bt3 + points[i+2].Y*bt2 + points[i+3].Y*bt1;
        newPoint.Z = points[i].Z*bt4 + points[i+1].Z*bt3 + points[i+2].Z*bt2 + points[i+3].Z*bt1;
        curve.push_back(newPoint);
      }
   }

   return curve;
}

Point3D
BSpline::predictPoint(int i, double t, std::vector<Point3D> points){
    Point3D point;
    point.X = 0; point.Y = 0; point.Z = 0;
    for (int j = -2; j <= 1; j++) {
        point.X += points[i + j].X * baseFunction(j, t);
        point.Y += points[i + j].Y * baseFunction(j, t);
        point.Z += points[i + j].Z * baseFunction(j, t);
    }
    return point;
}
