#include "BSpline.h"

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
