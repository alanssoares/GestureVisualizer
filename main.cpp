//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "util/ConstantsUtil.h"
#include "util/Util.h"
#include "util/FileUtil.h"

using pcl::PointCloud;
using pcl::PointXYZ;

Util g_Util;
std::vector<type_gesture> m_AllGestures;
std::vector<type_gesture> g_Gestures;
std::vector<pcl::PointXYZRGB> g_PointsNormalA, g_PointsNormalB, g_PointsProcessedA, g_PointsProcessedB;
std::string g_IdCloudA = "cloudA", g_IdCloudB = "cloudB", g_NameMethodCombination;
std::vector<pcl::visualization::Camera> g_Cam;
int g_IdView1(0), g_IdView2(0), g_np = 1, id_Gesture = 0, g_Methods = 1;
double g_curvature_threshold = g_Util.m_CurvThreshold;

void loadAll(){
	FileUtil& futil = FileUtil::getInstance();
	futil.loadGestures(NAME_FILE_DATA_NORMALIZED);
	m_AllGestures.clear();
	m_AllGestures.reserve(futil.mGesturesOneHand.size() + futil.mGesturesTwoHands.size());
	m_AllGestures.insert( m_AllGestures.end(), futil.mGesturesOneHand.begin(), futil.mGesturesOneHand.end() );
	m_AllGestures.insert( m_AllGestures.end(), futil.mGesturesTwoHands.begin(), futil.mGesturesTwoHands.end() );
	std::sort(m_AllGestures.begin(), m_AllGestures.end(), MathUtil::sortByName);
}

std::vector<pcl::PointXYZRGB> converterToPointXYZ(std::vector<Point3D> points){
  std::vector<pcl::PointXYZRGB> pointsConverted;
  pcl::PointXYZRGB newPoint;
  size_t n = points.size();
  for (int i = 0; i < n; i++){
    newPoint.x = points[i].X;
    newPoint.y = points[i].Y;
    newPoint.z = points[i].Z;
    pointsConverted.push_back(newPoint);
  }
  return pointsConverted;
}

void clearAllVectores(){
  g_PointsNormalA.clear();
  g_PointsNormalB.clear();
  g_PointsProcessedA.clear();
  g_PointsProcessedB.clear();
}

void removeAll(pcl::visualization::PCLVisualizer *viewer){
  viewer->removeAllShapes(g_IdView1);
  viewer->removeAllShapes(g_IdView2);
  viewer->removePointCloud(g_IdCloudA, g_IdView1);
  viewer->removePointCloud(g_IdCloudB, g_IdView2);
}

Point3D converterToPoint3D(pcl::PointXYZRGB point){
    Point3D newPoint;
    newPoint.X = point.x;
    newPoint.Y = point.y;
    newPoint.Z = point.z;
    return newPoint;
}

float calcCurvature(pcl::PointXYZRGB a, pcl::PointXYZRGB b, pcl::PointXYZRGB c){
    return MathUtil::calcCurvature(converterToPoint3D(a), converterToPoint3D(b), converterToPoint3D(c));
}

void improveCurrentGesture(){

  g_PointsNormalA = converterToPointXYZ(g_Gestures[id_Gesture].handOne.positions);
  g_PointsNormalB = converterToPointXYZ(g_Gestures[id_Gesture].handTwo.positions);

  if(g_Methods == 1){
    g_NameMethodCombination = "Laplacian + DouglasPeucker";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions), g_Util.m_DougThreshold, false));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions), g_Util.m_DougThreshold, false));
  } else if(g_Methods == 2) {
    g_NameMethodCombination = "B-Spline + DouglasPeucker";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(BSpline::uniformFitting(g_Gestures[id_Gesture].handOne.positions), g_Util.m_DougThreshold, false));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(BSpline::uniformFitting(g_Gestures[id_Gesture].handTwo.positions), g_Util.m_DougThreshold, false));
  } else if(g_Methods == 3) {
    g_NameMethodCombination = "Laplacian + Curvature";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions), g_curvature_threshold));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions), g_curvature_threshold));
  } else if(g_Methods == 4) {
    g_NameMethodCombination = "B-Spline + Curvature";
		BSpline spline;
    g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(spline.compute(g_Gestures[id_Gesture].handOne.positions, 3, 0.01), g_curvature_threshold));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(spline.compute(g_Gestures[id_Gesture].handTwo.positions, 3, 0.01), g_curvature_threshold));
  } else if(g_Methods == 5) {
    g_NameMethodCombination = "DouglasPeucker";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(g_Gestures[id_Gesture].handOne.positions, g_Util.m_DougThreshold, false));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(g_Gestures[id_Gesture].handTwo.positions, g_Util.m_DougThreshold, false));
  } else if(g_Methods == 6) {
    g_NameMethodCombination = "Curvature";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(g_Gestures[id_Gesture].handOne.positions, g_curvature_threshold));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(g_Gestures[id_Gesture].handTwo.positions, g_curvature_threshold));
  } else if(g_Methods == 7) {
    g_NameMethodCombination = "B-Spline";
		BSpline spline;
		g_PointsProcessedA = converterToPointXYZ(spline.compute(g_Gestures[id_Gesture].handOne.positions, 3, 0.01));
		g_PointsProcessedB = converterToPointXYZ(spline.compute(g_Gestures[id_Gesture].handTwo.positions, 3, 0.01));
  } else if(g_Methods == 8) {
    g_NameMethodCombination = "Laplacian";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions));
  } else if (g_Methods == 9) {
    g_NameMethodCombination = "Normalized between -1 and 1";
    Point3D max = MathUtil::findMaxFromTwo(g_Gestures[id_Gesture].handOne.positions, g_Gestures[id_Gesture].handTwo.positions);
    Point3D min = MathUtil::findMinFromTwo(g_Gestures[id_Gesture].handOne.positions, g_Gestures[id_Gesture].handTwo.positions);
    g_PointsProcessedA = converterToPointXYZ(MathUtil::normalizeTrajectory(g_Gestures[id_Gesture].handOne.positions, min, max));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::normalizeTrajectory(g_Gestures[id_Gesture].handTwo.positions, min, max));
  }
 }

void viewLabels(pcl::visualization::PCLVisualizer *viewer){
	int x = 10, y = 10;

	viewer->addText("Name : " + g_Gestures[id_Gesture].name, x, y + 10, "v1 text", g_IdView1);
	viewer->addText("Name : " + g_Gestures[id_Gesture].name, x, y + 10, "v2 text", g_IdView2);
  viewer->addText("Nº Points: " + MathUtil::intToString(g_PointsNormalA.size()), x, y, "v3 text", g_IdView1);
	viewer->addText("Nº Points: " + MathUtil::intToString(g_PointsProcessedA.size()), x, y, "v4 text", g_IdView2);

	if (g_Methods == 3 || g_Methods == 4 || g_Methods == 6) {
		viewer->addText("Threshold Curvature: " + MathUtil::floatToString(g_curvature_threshold), x, y + 20, "v5 text", g_IdView2);
		viewer->addText("Method: " + g_NameMethodCombination, x, y + 30, "v6 text", g_IdView2);
	} else if (g_Methods == 1 || g_Methods == 2 || g_Methods == 5) {
		viewer->addText("Threshold DouglasPeucker: " + MathUtil::floatToString(g_Util.m_DougThreshold), x, y + 20, "v7 text", g_IdView2);
		viewer->addText("Method: " + g_NameMethodCombination, x, y + 30, "v8 text", g_IdView2);
	} else {
		viewer->addText("Method: " + g_NameMethodCombination, x, y + 20, "v9 text", g_IdView2);
	}
}

void viewNormalGesture(pcl::visualization::PCLVisualizer *viewer){
  size_t nA = g_PointsNormalA.size();
	Point3D color;
	color.r = 1.0; color.g = 0.0; color.b = 0.0;
  for (int i = 0; i < nA - 1; i ++){
		std::ostringstream os;
		os << "line_normal_a_" << color.r << "_" << color.g << "_" << color.b << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsNormalA[i], g_PointsNormalA[i + 1], color.r, color.g, color.b, os.str(), g_IdView1);
  }
  size_t nB = g_PointsNormalB.size();
  for (int i = 0; i < nB - 1; i++){
		std::ostringstream os;
		os << "line_normal_b_" << color.r << "_" << color.g << "_" << color.b << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsNormalB[i], g_PointsNormalB[i + 1], color.r, color.g, color.b, os.str(), g_IdView1);
  }
}

void viewProcessedGesture(pcl::visualization::PCLVisualizer *viewer){
  size_t nA = g_PointsProcessedA.size();
	Point3D color;
	double curv = 0.0;
  for (int i = 0; i < nA - 2; i += 2){
		curv = MathUtil::calcCurvature(
			converterToPoint3D(g_PointsProcessedA[i]),
			converterToPoint3D(g_PointsProcessedA[i + 1]),
			converterToPoint3D(g_PointsProcessedA[i + 2]));
		// std::cout << "C = " << curv << '\n';
		if (curv <= 0) {
		// if (curv < g_curvature_threshold) {
			color.r = 0.0; color.g = 0.0; color.b = 1.0;
		} else {
			color.r = 0.0; color.g = 1.0; color.b = 0.0;
		}
		std::ostringstream os;
		os << "line_pros_a_" << color.r << "_" << color.g << "_" << color.b << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsProcessedA[i], g_PointsProcessedA[i + 1], color.r, color.g, color.b, os.str(), g_IdView2);
		os << "line_pros_a_" << color.r << "_" << color.g << "_" << color.b << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsProcessedA[i + 1], g_PointsProcessedA[i + 2], color.r, color.g, color.b, os.str(), g_IdView2);
  }
  size_t nB = g_PointsProcessedB.size();
  for (int i = 0; i < nB - 2; i += 2){
		curv = MathUtil::calcCurvature(
			converterToPoint3D(g_PointsProcessedB[i]),
			converterToPoint3D(g_PointsProcessedB[i + 1]),
			converterToPoint3D(g_PointsProcessedB[i + 2]));
		// std::cout << "C = " << curv << '\n';
		if (curv <= 0) {
		// if (curv < g_curvature_threshold) {
			color.r = 0.0; color.g = 0.0; color.b = 1.0;
		} else {
			color.r = 0.0; color.g = 1.0; color.b = 0.0;
		}
		std::ostringstream os;
		os << "line_pros_b_" << color.r << "_" << color.g << "_" << color.b << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsProcessedB[i], g_PointsProcessedB[i + 1], color.r, color.g, color.b, os.str(), g_IdView2);
		// os << "line_pros_b_" << i;
		// viewer->addLine<pcl::PointXYZRGB>(g_PointsProcessedB[i + 1], g_PointsProcessedB[i + 2], color.r, color.g, color.b, os.str(), g_IdView2);
  }
}

void viewShapes(pcl::visualization::PCLVisualizer *viewer){
  //Clear all global declared vectores
  clearAllVectores();
  //Remove all shapes, lines, etc from the view
  removeAll(viewer);
  //Reload all gestures
  g_Gestures = m_AllGestures;
  //Transform and process the gestures of the viewport 1 and 2
  improveCurrentGesture();
  //Plot all labels in the screen
  viewLabels(viewer);
  //Plot the normal gesture
  viewNormalGesture(viewer);
  //Plot the processed gesture
  viewProcessedGesture(viewer);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

  if(event.keyDown()){
    size_t n = g_Gestures.size() - 1;
    if(event.getKeySym() == "t"){
        g_curvature_threshold += g_Util.m_CurvThreshold;
        g_Util.m_DougThreshold += 0.01;
    } else if(event.getKeySym() == "y"){
        if(g_curvature_threshold > g_Util.m_CurvThreshold) g_curvature_threshold -= g_Util.m_CurvThreshold;
        if(g_Util.m_DougThreshold > 0.01) g_Util.m_DougThreshold -= 0.01;
    } else if(event.getKeySym() == "n" && id_Gesture > 0){
        id_Gesture -= 1;
    } else if(event.getKeySym() == "m" && id_Gesture < n){
        id_Gesture += 1;
    }
    viewShapes(viewer);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewCurvesVis()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Curve Viewer"));
  pcl::PointXYZ cameraPos, cameraView, cameraFocal;

  loadAll();

  viewer->initCameraParameters ();

  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, g_IdView1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, g_IdView2);

  viewShapes(viewer.get());

  viewer->setBackgroundColor (0, 0, 0, g_IdView1);
  viewer->setBackgroundColor (0, 0, 0, g_IdView2);

  viewer->addCoordinateSystem (1.0);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

  cameraPos.x = 2.32234; cameraPos.y = 4.85785; cameraPos.z = 1.32948;
  cameraView.x = -0.0467694; cameraView.y = -0.0729935; cameraView.z = 0.996235;
  cameraFocal.x = 0.431872; cameraFocal.y = -0.250526; cameraFocal.z = 0.866443;
  viewer->setCameraPosition(cameraPos.x, cameraPos.y, cameraPos.z,
                            cameraView.x, cameraView.y, cameraView.z,
                            cameraFocal.x, cameraFocal.y, cameraFocal.z);
  return (viewer);
}

int helpUsage()
{
    PRINT(" ------- Command line input -------- ");
    PRINT("Usage: ./start options");
    PRINT("options: ");
    PRINT("");
    PRINT("-m : ");
    PRINT(" 1 - Laplacian + DouglasPeucker");
    PRINT(" 2 - B-Spline + DouglasPeucker");
    PRINT(" 3 - Laplacian + Curvature");
    PRINT(" 4 - B-Spline + Curvature");
    PRINT(" 5 - DouglasPeucker");
    PRINT(" 6 - Curvature");
    PRINT(" 7 - B-Spline");
    PRINT(" 8 - Laplacian");
    PRINT(" 9 - Equidistant using ArcLength");
    PRINT(" default - Normalized between -1 and 1");
    PRINT("");
    PRINT(" ------- KeyboardEvent controls ------- ");
    PRINT("t : Increment Threshold (Curvature and DouglasPeucker)");
    PRINT("y : Decrement Threshold (Curvature and DouglasPeucker)");
    PRINT("n : Previuos Gesture");
    PRINT("m : Next Gesture");
    PRINT("");
    PRINT("Example (1): ./start -m 1");

    return 1;
}

void PrintCamera(){
  cout << "Cam: " << endl
               << " - pos: (" << g_Cam[0].pos[0] << ", "    << g_Cam[0].pos[1] << ", "    << g_Cam[0].pos[2] << ")" << endl
               << " - view: ("    << g_Cam[0].view[0] << ", "   << g_Cam[0].view[1] << ", "   << g_Cam[0].view[2] << ")"    << endl
               << " - focal: ("   << g_Cam[0].focal[0] << ", "  << g_Cam[0].focal[1] << ", "  << g_Cam[0].focal[2] << ")"   << endl;
}

int main(int argc, char* argv[])
{

  if(pcl::console::find_argument (argc, argv, "-h") >= 0){
    helpUsage();
    return 0;
  }

  if(pcl::console::find_argument (argc, argv, "-m") >= 0){
    g_Methods = atoi(argv[2]);
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = viewCurvesVis();
  while (!viewer->wasStopped ()){
    viewer->spinOnce (100);
    viewer->getCameras(g_Cam);
  }

  return 0;
}
