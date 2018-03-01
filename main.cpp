//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>
#include <limits>

#include <glm/vec3.hpp> 
#include <glm/vec4.hpp> 
#include <glm/mat4x4.hpp> 
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/type_ptr.hpp> 

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "util/ConstantsUtil.h"
#include "util/Util.h"
#include "util/FileUtil.h"

#include "util/initShaders.h"
#include "util/Arcball.h"


GLuint  shader;

GLuint  axisVBO[3];
GLuint  gestureOriVBO[4];
GLuint  gestureProcVBO[4];
GLuint  numGestureOriVBO_A, 
        numGestureOriVBO_B,
        numGestureProcVBO_A,
        numGestureProcVBO_B;

bool    drawRef     = true,
        drawGesture = true,
        redraw      = true;

float   angleX      = 0.0f,
        angleY      = 0.0f,
        angleZ      = 0.0f;

Util g_Util;
std::vector<type_gesture>   m_AllGestures;
std::vector<type_gesture>   g_Gestures;

//std::vector<pcl::Point3D> g_PointsNormalA, g_PointsNormalB, g_PointsProcessedA, g_PointsProcessedB;
std::vector<Point3D>    g_PointsNormalA, 
                        g_PointsNormalB, 
                        g_PointsProcessedA, 
                        g_PointsProcessedB;

std::string             g_IdCloudA = "cloudA", 
                        g_IdCloudB = "cloudB", 
                        g_NameMethodCombination;

int     g_IdView1(0), 
        g_IdView2(0), 
        g_np        = 1, 
        id_Gesture  = 0, 
        g_Methods   = 1,
        w,
        h;

double  g_curvature_threshold = g_Util.m_CurvThreshold,
        g_min_curvature = std::numeric_limits<double>::max(),
        g_max_curvature = -std::numeric_limits<double>::max();

static Arcball arcball( 800, 400, 1.5f, true, true );

 
/// ***********************************************************************
/// **
/// ***********************************************************************

void loadAll() {

	FileUtil& futil = FileUtil::getInstance();

	futil.loadGestures(NAME_FILE_DATA_NORMALIZED);

	g_Gestures.clear();
	g_Gestures.reserve(futil.mGesturesOneHand.size() + futil.mGesturesTwoHands.size());
	g_Gestures.insert( g_Gestures.end(), futil.mGesturesOneHand.begin(), futil.mGesturesOneHand.end() );
	g_Gestures.insert( g_Gestures.end(), futil.mGesturesTwoHands.begin(), futil.mGesturesTwoHands.end() );

	std::sort(g_Gestures.begin(), g_Gestures.end(), MathUtil::sortByName);

  cout << "number of Gestures:" << endl;
  cout << "   One Hand  = " << futil.mGesturesOneHand.size() << endl;
  cout << "   Two Hands = " << futil.mGesturesTwoHands.size() << endl;
  cout << "   Total     = " << m_AllGestures.size() << endl;
}

/// ***********************************************************************
/// **
/// ***********************************************************************

std::vector<Point3D> converterToPointXYZ(std::vector<Point3D> points) {

  std::vector<Point3D> pointsConverted;

  Point3D newPoint;

  size_t n = points.size();

  for (int i = 0; i < n; i++) {
    newPoint.X = points[i].X;
    newPoint.Y = points[i].Y;
    newPoint.Z = points[i].Z;
    pointsConverted.push_back(newPoint);
    }

  for (int i = 0; i < n; i++) {
    pointsConverted[i].r = 0.0f;
    pointsConverted[i].g = 1.0f;
    pointsConverted[i].b = 1.0f;
    }

  return pointsConverted;
}
 
/// ***********************************************************************
/// **
/// ***********************************************************************

float calcCurvature(Point3D a, Point3D b, Point3D c) {

    return MathUtil::calcCurvature(a, b, c);
}
 
/// ***********************************************************************
/// **
/// ***********************************************************************

GLuint buildVBONormalGesture(std::vector<Point3D> ptos, GLuint vVBO, GLuint cVBO) {

  size_t n = ptos.size();

  std::vector<double> vertVBO;
  std::vector<double> colorVBO;

  g_min_curvature = std::numeric_limits<double>::max();
  g_max_curvature = -std::numeric_limits<double>::max();

  for (int i = 0; i < n - 1; i ++) {
    
    vertVBO.push_back(ptos[i].X);
    vertVBO.push_back(ptos[i].Y);
    vertVBO.push_back(ptos[i].Z);

    double curv = MathUtil::calcCurvature ( ptos[i + 0],
                                            ptos[i + 1],
                                            ptos[i + 2]
                                          );

    ptos[i].curvature = curv;

    if ( curv < g_min_curvature )
      g_min_curvature = curv;

    if ( curv > g_max_curvature )
      g_max_curvature = curv;
    }

  double range = g_max_curvature - g_min_curvature;

  for (int i = 0; i < n - 1; i ++) {

    ptos[i].r = (ptos[i].curvature - g_min_curvature) / range;
    ptos[i].g = 1.0 - (ptos[i].curvature - g_min_curvature) / range;
    ptos[i].b = 1.0;

    colorVBO.push_back(ptos[i].X);
    colorVBO.push_back(ptos[i].Y);
    colorVBO.push_back(ptos[i].Z);
    }

  assert(colorVBO.size() == vertVBO.size());
  
  glBindBuffer( GL_ARRAY_BUFFER, vVBO);

  glBufferData( GL_ARRAY_BUFFER, sizeof(double)*vertVBO.size(), 
                vertVBO.data(), GL_STATIC_DRAW);

  glBindBuffer( GL_ARRAY_BUFFER, cVBO);

  glBufferData( GL_ARRAY_BUFFER, sizeof(float)*colorVBO.size(), 
                colorVBO.data(), GL_STATIC_DRAW);

  vertVBO.clear();
  colorVBO.clear();

  return n;
}

/// ***********************************************************************
/// **
/// ***********************************************************************

void improveCurrentGesture() {
  
  BSpline spline;

  switch (g_Methods) {

    case 1  : g_NameMethodCombination = "Laplacian + DouglasPeucker";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions), g_Util.m_DougThreshold, false));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions), g_Util.m_DougThreshold, false));
              break;

    case 2  : g_NameMethodCombination = "B-Spline + DouglasPeucker";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(BSpline::uniformFitting(g_Gestures[id_Gesture].handOne.positions), g_Util.m_DougThreshold, false));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(BSpline::uniformFitting(g_Gestures[id_Gesture].handTwo.positions), g_Util.m_DougThreshold, false));
              break;

    case 3  : g_NameMethodCombination = "Laplacian + Curvature";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions), g_curvature_threshold));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions), g_curvature_threshold));
              break;

    case 4  : g_NameMethodCombination = "B-Spline + Curvature";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(spline.compute(g_Gestures[id_Gesture].handOne.positions, 3, 0.01), g_curvature_threshold));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(spline.compute(g_Gestures[id_Gesture].handTwo.positions, 3, 0.01), g_curvature_threshold));
              break;

    case 5  : g_NameMethodCombination = "DouglasPeucker";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::simplify(g_Gestures[id_Gesture].handOne.positions, g_Util.m_DougThreshold, false));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::simplify(g_Gestures[id_Gesture].handTwo.positions, g_Util.m_DougThreshold, false));
              break;

    case 6  : g_NameMethodCombination = "Curvature";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::reduceByCurvature(g_Gestures[id_Gesture].handOne.positions, g_curvature_threshold));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::reduceByCurvature(g_Gestures[id_Gesture].handTwo.positions, g_curvature_threshold));
              break;

    case 7  : g_NameMethodCombination = "B-Spline";
		          g_PointsProcessedA = converterToPointXYZ(spline.compute(g_Gestures[id_Gesture].handOne.positions, 3, 0.01));
		          g_PointsProcessedB = converterToPointXYZ(spline.compute(g_Gestures[id_Gesture].handTwo.positions, 3, 0.01));
              break;

    case 8  : g_NameMethodCombination = "Laplacian";
              g_PointsProcessedA = converterToPointXYZ(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handOne.positions));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::smoothMeanNeighboring(g_Gestures[id_Gesture].handTwo.positions));
              break;

    case 9  : g_NameMethodCombination = "Normalized between -1 and 1";
              Point3D max = MathUtil::findMaxFromTwo(g_Gestures[id_Gesture].handOne.positions, g_Gestures[id_Gesture].handTwo.positions);
              Point3D min = MathUtil::findMinFromTwo(g_Gestures[id_Gesture].handOne.positions, g_Gestures[id_Gesture].handTwo.positions);
              g_PointsProcessedA = converterToPointXYZ(MathUtil::normalizeTrajectory(g_Gestures[id_Gesture].handOne.positions, min, max));
              g_PointsProcessedB = converterToPointXYZ(MathUtil::normalizeTrajectory(g_Gestures[id_Gesture].handTwo.positions, min, max));
              break;
    }

  numGestureProcVBO_A = buildVBONormalGesture(g_PointsProcessedA, gestureProcVBO[0], gestureProcVBO[1]);
  numGestureProcVBO_B = buildVBONormalGesture(g_PointsProcessedB, gestureProcVBO[2], gestureProcVBO[3]);
}
 
/// ***********************************************************************
/// **
/// ***********************************************************************

void viewGesture(GLuint vbo[], GLuint nA, GLuint nB) {

int attrV, attrC; 
  
  glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);    
  attrV = glGetAttribLocation(shader, "aPosition");
  glVertexAttribPointer(attrV, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrV);

  glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);    
  attrC = glGetAttribLocation(shader, "aColor");
  glVertexAttribPointer(attrC, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrC);

  glDrawElements(GL_LINE_STRIP, nA, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

  glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);    
  attrV = glGetAttribLocation(shader, "aPosition");
  glVertexAttribPointer(attrV, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrV);

  glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);    
  attrC = glGetAttribLocation(shader, "aColor");
  glVertexAttribPointer(attrC, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrC);

  glDrawElements(GL_LINE_STRIP, nB, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

  glDisableVertexAttribArray(attrV);
  glDisableVertexAttribArray(attrC);

  glBindBuffer(GL_ARRAY_BUFFER, 0); 
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); 


}

/// ***********************************************************************
/// **
/// ***********************************************************************

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
 
/// ***********************************************************************
/// **
/// ***********************************************************************

void createAxis() {
    
GLfloat vertices[]  =   {   0.0, 0.0, 0.0,
                            2.0, 0.0, 0.0,
                            0.0, 2.0, 0.0,
                            0.0, 0.0, 2.0
                        }; 

GLuint lines[]  =   {   0, 1,
                        0, 2,
                        0, 3
                    }; 

GLfloat colors[]  = {   1.0, 1.0, 1.0, 1.0,
                        1.0, 0.0, 0.0, 1.0,
                        0.0, 1.0, 0.0, 1.0,
                        0.0, 0.0, 1.0, 1.0
                    }; 

  
  
  glGenBuffers(3, axisVBO);

  glBindBuffer( GL_ARRAY_BUFFER, axisVBO[0]);

  glBufferData( GL_ARRAY_BUFFER, 4*3*sizeof(float), 
                vertices, GL_STATIC_DRAW);

  glBindBuffer( GL_ARRAY_BUFFER, axisVBO[1]);

  glBufferData( GL_ARRAY_BUFFER, 4*4*sizeof(float), 
                colors, GL_STATIC_DRAW);

  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, axisVBO[2]);

  glBufferData( GL_ELEMENT_ARRAY_BUFFER, 3*2*sizeof(unsigned int), 
                lines, GL_STATIC_DRAW);

}
      
/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

void drawAxis() {

int attrV, attrC; 
  
  glBindBuffer(GL_ARRAY_BUFFER, axisVBO[0]);    
  attrV = glGetAttribLocation(shader, "aPosition");
  glVertexAttribPointer(attrV, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrV);

  glBindBuffer(GL_ARRAY_BUFFER, axisVBO[1]);    
  attrC = glGetAttribLocation(shader, "aColor");
  glVertexAttribPointer(attrC, 4, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(attrC);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, axisVBO[2]);
  glDrawElements(GL_LINES, 6, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

  glDisableVertexAttribArray(attrV);
  glDisableVertexAttribArray(attrC);

  glBindBuffer(GL_ARRAY_BUFFER, 0); 
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); 
}

/// ***********************************************************************
/// **
/// ***********************************************************************

void viewShapes() {

  glViewport(0, 0, w/2, h);
  viewGesture(gestureOriVBO, numGestureOriVBO_A, numGestureOriVBO_B);

  glViewport(w/2, 0, w/2, h);
  viewGesture(gestureProcVBO, numGestureProcVBO_A, numGestureProcVBO_B);
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

void display(void) { 

//  angleX += 0.05;
//  angleY += 0.02;
//  angleZ += 0.001;
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

  glm::mat4 P   = glm::perspective( 70.0, 1.0, 0.01, 100.0);
  glm::mat4 V   = glm::lookAt ( glm::vec3(2.0,  2.0, 2.0),
                                glm::vec3(0.0, 0.0, 0.0), 
                                glm::vec3(0.0, 1.0, 0.0) 
                              );

  glm::mat4 M   = glm::mat4(1.0);

  glm::mat4 rotated_view = V * arcball.createViewRotationMatrix();
//  glm::mat4 mvp = projection * rotated_view * model;


  M = glm::rotate( M, angleX, glm::vec3(1.0, 0.0, 0.0));
  M = glm::rotate( M, angleY, glm::vec3(0.0, 1.0, 0.0));
  M = glm::rotate( M, angleZ, glm::vec3(0.0, 0.0, 1.0));

  glm::mat4 MVP = P * rotated_view * M;

  glUseProgram(shader);

  int loc = glGetUniformLocation( shader, "uMVP" );
  glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(MVP));
  
  if (drawRef) {
    glViewport(0, 0, w/2, h);
    drawAxis();
    glViewport(w/2, 0, w/2, h);
    drawAxis();
    }

  glUseProgram(0);  

  if (drawGesture) 
    viewShapes();
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

static void error_callback(int error, const char* description) {
    cout << "Error: " << description << endl;
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

static void window_size_callback(GLFWwindow* window, int width, int height) {
    cout << "Resized window: " << width << " , " << height << endl;
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

void mouseButtonCallback( GLFWwindow * window, int button, int action, int mods ){
    /* Pass the arguments to our arcball object */
    arcball.mouseButtonCallback( window, button, action, mods );
    redraw = true;
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

void cursorCallback( GLFWwindow *window, double x, double y ) {
    /* Pass the arguments to our arcball object */
    arcball.cursorCallback( window, x, y );
    redraw = true;
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {

  size_t n = g_Gestures.size() - 1;

  if (action == GLFW_PRESS) {                  
    switch (key) {  
      case GLFW_KEY_ESCAPE    : glfwSetWindowShouldClose(window, GLFW_TRUE);
                                break;

      case 'T'                : 
      case 't'                : g_curvature_threshold += g_Util.m_CurvThreshold;
                                g_Util.m_DougThreshold += 0.01;
                                break;
      case 'Y'                : 
      case 'y'                : if(g_curvature_threshold > g_Util.m_CurvThreshold) 
                                  g_curvature_threshold -= g_Util.m_CurvThreshold;

                                if(g_Util.m_DougThreshold > 0.01) 
                                  g_Util.m_DougThreshold -= 0.01;

                                break;
      case 'N'                : 
      case 'n'                : if (id_Gesture > 0) {
                                  id_Gesture -= 1;
                                  g_PointsNormalA = converterToPointXYZ(g_Gestures[id_Gesture].handOne.positions);
                                  g_PointsNormalB = converterToPointXYZ(g_Gestures[id_Gesture].handTwo.positions);

                                  numGestureOriVBO_A = buildVBONormalGesture(g_PointsNormalA, gestureOriVBO[0], gestureOriVBO[1]);
                                  numGestureOriVBO_B = buildVBONormalGesture(g_PointsNormalB, gestureOriVBO[2], gestureOriVBO[3]);
                                  improveCurrentGesture();
                                  }
                                break;
      case 'M'                : 
      case 'm'                : if (id_Gesture < n) {
                                  id_Gesture += 1;
                                  g_PointsNormalA = converterToPointXYZ(g_Gestures[id_Gesture].handOne.positions);
                                  g_PointsNormalB = converterToPointXYZ(g_Gestures[id_Gesture].handTwo.positions);

                                  numGestureOriVBO_A = buildVBONormalGesture(g_PointsNormalA, gestureOriVBO[0], gestureOriVBO[1]);
                                  numGestureOriVBO_B = buildVBONormalGesture(g_PointsNormalB, gestureOriVBO[2], gestureOriVBO[3]);
                                  improveCurrentGesture();
                                  }
                                break;

      case '1'                : g_Methods = 1;
                                improveCurrentGesture();
                                break;

      case '2'                : g_Methods = 2;
                                improveCurrentGesture();
                                break;

      case '3'                : g_Methods = 3;
                                improveCurrentGesture();
                                break;

      case '4'                : g_Methods = 4;
                                improveCurrentGesture();
                                break;

      case '5'                : g_Methods = 5;
                                improveCurrentGesture();
                                break;

      case '6'                : g_Methods = 6;
                                improveCurrentGesture();
                                break;

      case '7'                : g_Methods = 7;
                                improveCurrentGesture();
                                break;

      case '8'                : g_Methods = 8;
                                improveCurrentGesture();
                                break;

      case '9'                : g_Methods = 9;
                                improveCurrentGesture();
                                break;
    }
  redraw = true;
  }

}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

static GLFWwindow* initGLFW(char* nameWin, int w, int h) {

  glfwSetErrorCallback(error_callback);

  if (!glfwInit())
      exit(EXIT_FAILURE);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  GLFWwindow* window = glfwCreateWindow(w, h, nameWin, NULL, NULL);
  if (!window) {
      glfwTerminate();
      exit(EXIT_FAILURE);
    }

  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetWindowSizeCallback(window, window_size_callback);
  glfwSetKeyCallback(window, key_callback);
  glfwSetCursorPosCallback( window, cursorCallback );
  glfwSetMouseButtonCallback( window, mouseButtonCallback );

  glfwMakeContextCurrent(window);

  glfwSwapInterval(1);

  return (window);
}


/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

void initShaders(void) {

  // Load shaders and use the resulting shader program
  shader = InitShader( "../shaders/basicShader.vert", "../shaders/basicShader.frag" );
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

void initGL(GLFWwindow* window) {

  glClearColor(0.0, 0.0, 0.0, 0.0); 
  
  if (glewInit()) {
    cout << "Unable to initialize GLEW ... exiting" << endl;
    exit(EXIT_FAILURE);
    }
    
  cout << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << endl; 
  
  cout << "Opengl Version: " << glGetString(GL_VERSION) << endl;
  cout << "Opengl Vendor : " << glGetString(GL_VENDOR) << endl;
  cout << "Opengl Render : " << glGetString(GL_RENDERER) << endl;
  cout << "Opengl Shading Language Version : " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;

  glfwGetFramebufferSize(window, &w, &h);

  glPointSize(3.0);
  glEnable(GL_DEPTH_TEST);

  initShaders();

  glGenBuffers(4, gestureOriVBO);

  if (gestureOriVBO[0] == 0) {
    cout << "Error gestureOriVBO" << endl;
    }
    
  glGenBuffers(4, gestureProcVBO);

  if (gestureProcVBO[0] == 0) {
    cout << "Error gestureOriVBO" << endl;
    }
}

/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */

static void GLFW_MainLoop(GLFWwindow* window) {

   while (!glfwWindowShouldClose(window)) {

      if (redraw) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        display();

        glfwSwapBuffers(window);
        redraw = false;
        }

        glfwPollEvents();
      }
}

/* ************************************************************************* */
/* ************************************************************************* */
/*                                                                           */
/* ************************************************************************* */
/* ************************************************************************* */

int main(int argc, char* argv[]) {

  GLFWwindow* window;

  window = initGLFW(argv[0], 800, 400);

  loadAll();
  initGL(window);
  createAxis();

  GLFW_MainLoop(window);

  glfwDestroyWindow(window);
  glfwTerminate();

  exit(EXIT_SUCCESS);
}

