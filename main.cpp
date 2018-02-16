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
	viewer->removeAllShapes(0);
	viewer->removeAllShapes(1);
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
    g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions), g_Util.m_CurvThreshold));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions), g_Util.m_CurvThreshold));
  } else if(g_Methods == 4) {
    g_NameMethodCombination = "B-Spline + Curvature";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(BSpline::uniformFitting(g_Gestures[id_Gesture].handOne.positions), g_Util.m_CurvThreshold));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(BSpline::uniformFitting(g_Gestures[id_Gesture].handTwo.positions), g_Util.m_CurvThreshold));
  } else if(g_Methods == 5) {
    g_NameMethodCombination = "DouglasPeucker";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(g_Gestures[id_Gesture].handOne.positions, g_Util.m_DougThreshold, false));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(g_Gestures[id_Gesture].handTwo.positions, g_Util.m_DougThreshold, false));
  } else if(g_Methods == 6) {
    g_NameMethodCombination = "Curvature";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(g_Gestures[id_Gesture].handOne.positions, g_Util.m_CurvThreshold));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(g_Gestures[id_Gesture].handTwo.positions, g_Util.m_CurvThreshold));
  } else if(g_Methods == 7) {
    g_NameMethodCombination = "B-Spline";
    g_PointsProcessedA = converterToPointXYZ(BSpline::uniformFitting(g_Gestures[id_Gesture].handOne.positions));
    g_PointsProcessedB = converterToPointXYZ(BSpline::uniformFitting(g_Gestures[id_Gesture].handTwo.positions));
  } else if(g_Methods == 8) {
    g_NameMethodCombination = "Laplacian";
    g_PointsProcessedA = converterToPointXYZ(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions));
  } else if(g_Methods == 9) {
    g_NameMethodCombination = "Equidistant using ArcLength";
    type_gesture temp = g_Gestures[id_Gesture];
    MathUtil::uniformCurveByArcLength(&temp.handOne.positions, 0.1);
    MathUtil::uniformCurveByArcLength(&temp.handTwo.positions, 0.1);
    g_PointsProcessedA = converterToPointXYZ(temp.handOne.positions);
    g_PointsProcessedB = converterToPointXYZ(temp.handTwo.positions);
  } else {
    g_NameMethodCombination = "Normalized between -1 and 1";
    Point3D max = MathUtil::findMaxFromTwo(g_Gestures[id_Gesture].handOne.positions, g_Gestures[id_Gesture].handTwo.positions);
    Point3D min = MathUtil::findMinFromTwo(g_Gestures[id_Gesture].handOne.positions, g_Gestures[id_Gesture].handTwo.positions);
    g_PointsProcessedA = converterToPointXYZ(MathUtil::normalizeTrajectory(g_Gestures[id_Gesture].handOne.positions, min, max));
    g_PointsProcessedB = converterToPointXYZ(MathUtil::normalizeTrajectory(g_Gestures[id_Gesture].handTwo.positions, min, max));
  }
 }

void viewLabels(pcl::visualization::PCLVisualizer *viewer){
  viewer->addText("Nº Points: " + MathUtil::intToString(g_PointsNormalA.size()), 10, 10, "v1 text", g_IdView1);
  viewer->addText("Nº Total: " + MathUtil::intToString(g_Gestures.size()), 10, 20, "v2 text", g_IdView1);
  viewer->addText("Name : " + g_Gestures[id_Gesture].name, 10, 30, "v3 text", g_IdView1);
  viewer->addText("Id : " + MathUtil::intToString(id_Gesture), 10, 40, "v4 text", g_IdView1);

  viewer->addText("Nº Points: " + MathUtil::intToString(g_PointsProcessedA.size()), 10, 10, "v5 text", g_IdView2);
  viewer->addText("Threshold Curvature: " + MathUtil::floatToString(g_Util.m_CurvThreshold), 10, 20, "v6 text", g_IdView2);
  viewer->addText("Threshold DouglasPeucker: " + MathUtil::floatToString(g_Util.m_DougThreshold), 10, 30, "v7 text", g_IdView2);
  viewer->addText("Method: " + g_NameMethodCombination, 10, 40, "v8 text", g_IdView2);
}

void viewNormalGesture(pcl::visualization::PCLVisualizer *viewer){
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGB>);

  size_t nA = g_PointsNormalA.size();
  for (int i = 0; i < nA - 1; i++){
    // cloudA->points.push_back(g_PointsNormalA[i]);
		std::ostringstream os;
		os << "line_normal_a_" << 1.0 << "_" << 0.0 << "_" << 0.0 << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsNormalA[i], g_PointsNormalA[i + 1], 1.0, 0.0, 0.0, os.str(), 0);
  }

  size_t nB = g_PointsNormalB.size();
  for (int i = 0; i < nB - 1; i++){
    // cloudA->points.push_back(g_PointsNormalB[i]);
		std::ostringstream os;
		os << "line_normal_b_" << 1.0 << "_" << 0.0 << "_" << 0.0 << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsNormalB[i], g_PointsNormalB[i + 1], 1.0, 0.0, 0.0, os.str(), 0);
  }

  // viewer->addPointCloud<pcl::PointXYZRGB> (cloudA, g_IdCloudA, g_IdView1);
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, g_IdCloudA);
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, g_IdCloudA);
}

void viewProcessedGesture(pcl::visualization::PCLVisualizer *viewer){
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGB>);

  size_t nA = g_PointsProcessedA.size();
  for (int i = 0; i < nA - 1; i++){
    // cloudB->points.push_back(g_PointsProcessedA[i]);
		std::ostringstream os;
		os << "line_pros_a_" << 0.0 << "_" << 0.0 << "_" << 1.0 << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsProcessedA[i], g_PointsProcessedA[i + 1], 0.0, 0.0, 1.0, os.str(), 1);
  }

  size_t nB = g_PointsProcessedB.size();
  for (int i = 0; i < nB - 1; i++){
    // cloudB->points.push_back(g_PointsProcessedB[i]);
		std::ostringstream os;
		os << "line_pros_b_" << 0.0 << "_" << 0.0 << "_" << 1.0 << "_" << i;
		viewer->addLine<pcl::PointXYZRGB>(g_PointsProcessedB[i], g_PointsProcessedB[i + 1], 0.0, 0.0, 1.0, os.str(), 1);
  }

  // viewer->addPointCloud<pcl::PointXYZRGB> (cloudB, g_IdCloudB, g_IdView2);
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, g_IdCloudB);
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, g_IdCloudB);
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
        g_Util.m_CurvThreshold += 0.0001;
        g_Util.m_DougThreshold += 0.01;
    } else if(event.getKeySym() == "y"){
        if(g_Util.m_CurvThreshold > 0.0001) g_Util.m_CurvThreshold -= 0.0001;
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
