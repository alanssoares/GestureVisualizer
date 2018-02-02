//
//  MathUtil.h
//  GestureTracking
//
//  Created by Alan Santos on 22/10/15.
//  Copyright (c) 2015 Alan Santos. All rights reserved.
//

#ifndef __GestureTracking__MathUtil__
#define __GestureTracking__MathUtil__

#include <stdio.h>
#include <XnCppWrapper.h>
#include <vector>
#include <math.h>
#include <sstream>
#include "ConstantsUtil.h"
#include "BSpline.h"

using namespace std;

/*
 This class was created to support the development
 of Gesture Recognition Project based in methods that compare trajectories.
 */
class MathUtil {

public:

    static bool sortByName(const type_gesture &g1, const type_gesture &g2) { return g1.name < g2.name; }

    // Directions to search
  	static const int SEARCH_LEFT = 0;
	  static const int SEARCH_RIGHT = 1;
    static const int LOCAL_MIN = 0;
    static const int LOCAL_MAX = 1;
    static const int INFLECTION_POINT = 2;
    static const int END_POINT = 3;
    static const int ANY = 4;

    /**
     Método responsável por converter um int em string
    */
    static std::string intToString(int n);

    /**
     Método responsável por converter um float em string
    */
    static std::string floatToString(float n);

    /*
     http://www.shodor.org/~jmorrell/interactivate/org/shodor/util11/DataSetUtils.java
     http://math.stackexchange.com/questions/799783/slope-of-a-line-in-3d-coordinate-system
     http://www.ce.utexas.edu/prof/maidment/giswr2011/docs/Slope.pdf
     Calc the slop using the points p1 and p2
     @return double slop
    */
    static double calcSlope(XnPoint3D p1, XnPoint3D p2);

    /**
     Método responsável por calcular a amplitude/módulo escalar de um vetor.
     @param point da trajetória
     @return double
     */
    static double length(XnPoint3D point);

    /**
     Método responsável por calcular a distância entre dois pontos
     @param p1 ponto
     @param p2 ponto
     @return double distância entre os pontos
    **/
    static double getArcLength(XnPoint3D p1, XnPoint3D p2);

    /**
     Método responsável por efetuar a subtração de dois vetores a e b.
     @param a ponto da trajetória
     @param b ponto da trajetória
     @return XnPoint3D resultado da subtração
     */
    static XnPoint3D subtract(XnPoint3D a, XnPoint3D b);

    /**
     Método responsável por verificar se os pontos p1 e p2 são iguais.
     @param p1 ponto
     @param p2 ponto
     @retunr bool indicando se são iguais ou não
    **/
    static bool pointsEqual(XnPoint3D p1, XnPoint3D p2);

    /**
     Método responsável por normalizar um ponto utilizando a magnitude deste.
     Normaliza um vetor dividindo cada componente pelo módulo do vetor.
     @param point da trajetória
     @return XnPoint3D normalizado
     */
    static XnPoint3D normalize(XnPoint3D point);

    /**
     Método responsável por calcular a distância de um ponto p a um
     segmento formado pelos pontos p1 e p2.
     @param p ponto usado para calcular a distância
     @param p1 ponto do segmento
     @param p2 ponto do segmento
     @return double indicando a distância do ponto ao segmento
    **/
    static double getDistancePointToSegment(XnPoint3D p, XnPoint3D p1, XnPoint3D p2);

    /**
     Método responsável por obter os valores mínimos de X, Y, Z de dois vetores
     @param a vector 1
     @param b vector 2
     @return XnPoint3D com mínimos X,Y,Z
    */
    static XnPoint3D findMinFromTwo(vector<XnPoint3D> a, vector<XnPoint3D> b);

    /**
     Método responsável por obter os valores máximos de X, Y, Z de dois vetores
     @param a vector 1
     @param b vector 2
     @return XnPoint3D com máximos X,Y,Z
    */
    static XnPoint3D findMaxFromTwo(vector<XnPoint3D> a, vector<XnPoint3D> b);

    /**
     Método responsável por obter os valores mínimos de X, Y, Z
     do array de posições.
     @param positions da trajetória
     @return XnPoint3D valores mínimos de X, Y, Z
     */
    static XnPoint3D minValueXYZ(vector<XnPoint3D> positions);

    /**
     Método responsável por obter os valores máximos de X, Y, Z
     do array de posições da trajetória.
     @param positions da trajetória
     @return XnPoint3D valores máximos de X, Y, Z
     */
    static XnPoint3D maxValueXYZ(vector<XnPoint3D> positions);

    /*
     Método que calcula a interpolação linear entre dois pontos
     @param p0 ponto
     @param p1 ponto
     @param t instante
     @return XnPoint3D resultado da interpolação
    */
    static XnPoint3D interpolate(XnPoint3D p0, XnPoint3D p1, float t = 0.5);

    /**
     Método responsável por normalizar a trajetória no intervalo de -1 a 1
        newvalue= (max'-min')/(max-min)*(value-max)+max'
        or
        newvalue= (max'-min')/(max-min)*(value-min)+min'.
     @param positions da trajetória
     @param min values of X, Y, Z
     @param max values of X, Y, Z
     @return vector<XnPoint3D> normalizado
     */
    static vector<XnPoint3D> normalizeTrajectory(vector<XnPoint3D> positions, XnPoint3D min, XnPoint3D max);

    /**
     Método responsável por obter o ângulo entre dois pontos a e b.
     Calcula o ângulo formado entre dois vetores a e b, com
     uma reta saindo da origem e passando pelos pontos
     @param a posição da trajetória
     @param b posição da trajetória
     @return double representando o ângulo
     */
    static double getAngleBetween2Points(XnPoint3D a, XnPoint3D b);

    /**
     Método responsável por obter o valor máximo de um vetor
     @param values com valores
     @return double valor máximo encontrado
     */
    static double getMaxValue(std::vector<double> values);

    /**
     Método responsável por calcular o centro geométrico
     da trajetória, designado como centróide. O calculo
     é realizado através da razão entre somatório dos pontos
     e o número de pontos da trajetória. A centróide fornece
     a direção e distância para realizar a translação do movimento
     afim de obter invariância de posição.
     @param positions da trajetória
     @return XnPoint3D centróide encontrada a partir da trajetória
     */
    static XnPoint3D calcCentroid(vector<XnPoint3D> positions);

    /**
     Método responsável por realizar o deslocamento da trajetória para a
     origem utilizando a centróide da trajetória.
     Método responsável por normalizar a trajetória para reduzir
     o impacto de gestos realizados por pessoas com aspectos físicos
     diferentes. A normalização é realizada através de uma translação
     da trajetória para a origem (0,0,0) usando a centróide da trajetória.
     @param positions da trajetória
     @return vector<XnPoint3D> deslocada para a origem
     */
    static vector<XnPoint3D> translateToOrigin(vector<XnPoint3D> positions);

    /*
     Método responsável por normalizar a trajetória usando a média da primeira vizinhança, ou seja
     P(n - 1), P(n), P(n + 1).
     @param positions of the trajectory
     @return std::vector<XnPoint3D> da nova trajetória filtrada usando a média da primeira vizinhança
     */
    static std::vector<XnPoint3D> smoothMeanNeighboring(std::vector<XnPoint3D> positions);

    /**
     Método responsável por reduzir a quantidade de pontos da trajetória e
     manter as características da curva
     @ref http://mourner.github.io/simplify-js/
     @param points da trajetória
     @param tolerance da distância entre os pontos
     @param highestQuality que indica se será aplicado um algoritmo de distância radial
    */
    static std::vector<XnPoint3D> simplify(std::vector<XnPoint3D> points, double tolerance, bool highestQuality);

    /**
     Método responsável por aplicar o método de Douglas Peucker para simplificar a trajetória
     @param points a serem simplificados
     @param sqTolerance da distância entre dois pontos
     @return std::vector<XnPoint3D> com trajetória simplificada
    **/
    static std::vector<XnPoint3D> simplifyDouglasPeucker(std::vector<XnPoint3D> points, double sqTolerance);

    /**
     Método responsável por simplificar a trajetória usando a distância radial
     @param points da trajetória
     @param sqTolerance da distância entre dois pontos
     @return std::vector<XnPoint3D> com trajetória simplificada
    **/
    static std::vector<XnPoint3D> simplifyRadialDist(std::vector<XnPoint3D> points, double sqTolerance);

    /**
     Método responsável por aplicar um passo da simplificação de Douglas Peucker
     @param points da trajetória
     @param first ponto da trajetória
     @param last ponto da trajetória
     @param sqTolerence entre dois pontos
     @param simplified trajetória simplificada
    **/
    static void simplifyDPStep(std::vector<XnPoint3D> points, int first, int last, double sqTolerance, std::vector<XnPoint3D>* simplified);

    /**
    //P1, P2, P3
    //L1 = P2 - P1, L2 = P3 - P2
    //C = |L2 - L1|
    Método responsável por remover os pontos que possuem curvature menor que um threshold
    O objetivo é reduzir a quantidade de pontos a serem comparados e aumentar o desempenho
    sem perder qualidade na taxa de reconhecimento dos gestos.
    @param points a serem processados
    @param threshold
    @return std::vector<XnPoint3D> com trajetória simplificada
    */
    static std::vector<XnPoint3D> reduceByCurvature(std::vector<XnPoint3D> points, double threshold);

    /**
     Método responsável por calcular a curvatura utilizando 3 pontos
     @param a ponto i - 1
     @param b ponto i
     @param c ponto i + 1
     @return float com curvatura
    */
    static float calcCurvature(XnPoint3D a, XnPoint3D b, XnPoint3D c);

    /**
     Smooth the trajectory according with the method choosed
     @param trajectory
     @return std::vector<XnPoint3D>
    */
    static std::vector<XnPoint3D> smooth(std::vector<XnPoint3D> trajectory);

    /**
     Normalize (-1, 1) and center in the origin (0,0,0)
     @param trajectory
     @return std::vector<XnPoint3D>
    */
    static std::vector<XnPoint3D> normCenterOrigin(std::vector<XnPoint3D> trajectory);

    /**
     Smooth and reduce the points using the curvature
     @param trajectory
     @return std::vector<XnPoint3D>
    */
    static std::vector<XnPoint3D> smoothAndReduce(std::vector<XnPoint3D> trajectory);

    /*
     Method that sum two vector3f
     @param a as XnPoint3D
     @param b as XnPoint3D
     @return XnPoint3D
    */
    static XnPoint3D sum(XnPoint3D a, XnPoint3D b);

    /*
     Method that insert points according
     @param points original
     @param diff num of points that will be inserted
    */
    static void insertPoints(std::vector<XnPoint3D> *points, int diff);

    /*
     Method that remove points according
     @param points original
     @param diff num of points that will be removed
    */
    static void removePoints(std::vector<XnPoint3D> *points, int diff);

    /**
     Method that transform the curve in equidistant points using the arc length
     @param points original
     @param dL desired distance between points
    */
    static void uniformCurveByArcLength(std::vector<XnPoint3D> *points, double dL);
};

#endif /* defined(__GestureTracking__MathUtil__) */
