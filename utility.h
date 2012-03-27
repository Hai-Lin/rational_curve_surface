
#ifndef _UTILITY_H_
#define _UTILITY_H_
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include<stdio.h>
#include<stdlib.h>
#include"Vector3D.h"
#include"Vector2D.h"
using namespace std;

// This class stores the definition of a curve or surface. From this definition,
// you can create more appropriate data structures for your geometry, if need be.
//
// The parser below reads the data file, and generates a vector of this class, one
// element for each curve or surface in the file.
//
class curveOrSurface {
	public:

		curveOrSurface () : geoType (-1), degreeU (-1), degreeV (-1) {};
		void reset () { geoType = -1; degreeU = -1; degreeV = -1; knotsU.clear (); knotsV.clear (); controlPoints.clear ();};

		// these class members are referred to as fields during the parsing below.
		// geoType is field 0.
		// degreeU and degreeV are field 1 (they share a line in the input file)
		// knotsU is field 2
		// knotsV is field 3
		// controlpoints are field 4.

		int geoType;            // 0 for curve, 1 for surface
		int degreeU;            // has value for a curve or a surface
		int degreeV;            // has a value only for surfaces
		vector< float > knotsU; // has knots for both curves & surfaces
		vector< float > knotsV; // has knots (in V) only for surfaces
		vector< WVector3D > controlPoints;  // 
};

//construct a surface controlpoints for two curve swept with each other
void constructSwept(vector<curveOrSurface> &geoObjs); 

//construct a surface with controlpoints from one curve resolvation
void constructRevolved(vector<curveOrSurface> &geoObjs);

// parse the scene file whose names is given as the first argument.
// all the data should go into the geoObjs vector, which is a vector
// of curveOrSurface_s, each one representing a curve or surface read
// in from the file.
//
int parse (char *inFileName, vector< curveOrSurface > &geoObjs);


#endif
