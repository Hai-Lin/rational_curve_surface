//
// very minimal file parser for CAGD b-spline implementation homework.
//
// GO TO the MAIN function and see how this is used - you should probably 
// do this before looking at anything else.
//
// the parse() function reads the input file "test_input.txt" and loads in the
// data. you will likely need to re-arrange the data to more suit your needs,
// but at least you wont have to do the parsing! 
//

#ifndef _PARSER_H_
#define _PARSER_H_
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
/*
// super minimal point type:
class apoint {
public:
float &operator[] (const int &i) {return xyz[i];};
float xyz[3];
};
*/

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


// parse the scene file whose names is given as the first argument.
// all the data should go into the geoObjs vector, which is a vector
// of curveOrSurface_s, each one representing a curve or surface read
// in from the file.
//
inline int parse (char *inFileName, vector< curveOrSurface > &geoObjs)
{
	string line;

	ifstream myfile (inFileName);

	if ( myfile.is_open()) {

		int currentField = -1;

		curveOrSurface cOS; // temporary space for the current object we're parsing

		// when loading rows of control points, how many more do we have to read?
		// when this is -1, it's recalculated when the controlpoints field is
		// read:
		int nRowsLeft = -1;
		stringstream sstream;   // used for conversion and tokenizing
		while ( myfile.good()) {

			getline (myfile, line);

			// only examine the line if it's not blank, and not a comment
			if (line.find ("//") == string::npos && line.find_first_not_of(" \t\v\r\n") != string::npos ) {
				// we'll need this for some parsing in spots. it's sometimes 
				// not used:
				sstream.clear ();
				sstream.str (line);

				// object count in the file. ignored in this parser.
				if (currentField == -1) {
					// its the count of objects in this file. adds a little 
					// robustness.
				}

				// field that says this is a curve or surface:
				else if (currentField == 0) {
					// trimcurve, curve or surface?
					if (line.find ("trimcurve") != string::npos)
						cOS.geoType = 2;    // trim
					else if (line.find ("curve") != string::npos)
						cOS.geoType = 0;    // curve
					else if(line.find("surfaceswept")!=string::npos)
						constructSwept(geoObjs);
					else if(line.find("surfacerevolved")!=string::npos)
						constructRevolved(geoObjs);

					else
						cOS.geoType = 1;    // surface

				}
				/*	else if (currentField == 0) {
				// curve or surface?
				if (line.find ("curve") != string::npos)
				cOS.geoType = 0;    // curve
				else
				cOS.geoType = 1;    // surface
				}
				*/

				// field that specifies the degree in U (and possibly degree V)
				else if (currentField == 1) {
					sstream >> cOS.degreeU;

					if (cOS.geoType == 1) { // if it's a surface, get the other integer on this line
						sstream >> cOS.degreeV;
					}
				}

				// field that has the knot vector in U
				else if (currentField == 2) {
					float tempKnot;
					while (sstream >> tempKnot)
						cOS.knotsU.push_back (tempKnot);
				}

				// field that has the knot vector in V - only for surfaces
				else if (currentField == 3) {
					if (cOS.geoType == 0 || cOS.geoType == 2) {
						cout << "error: should not be adding V knots to a curve!" << endl;
						exit (-1);
					}
					float tempKnot;
					while (sstream >> tempKnot)
						cOS.knotsV.push_back (tempKnot);
				}

				// the control points, which span multiple columns and
				// multiple lines. we read one line at a time, then come
				// back through the loop for the next line, until done.
				else if (currentField == 4) { // control points

					WVector3D wPoint;
					Vector3D tempPoint;
					vector<Vector3D> pointBuffer;
					vector<float> weightBuffer;

					// how many rows & columns are we going to read in? assuming
					// the input is correct, we really just need to figure ot
					// how many rows - we can just read in all the columns as
					// consecutive x,y,z values for a control point.
					if (nRowsLeft == -1) {
						// we need to recaclulate it here:
						int nRows;
						if (cOS.geoType == 0 || cOS.geoType == 2) {
							// for curves, each control point is on a separate line.
							// there are 3 columns, 1 for each x, y, and z value.
							nRows = cOS.knotsU.size () - cOS.degreeU + 1;
						}
						else {
							// for surfaces, the number of rows is related to the
							// number of knots in V and the degree in V:
							nRows = cOS.knotsV.size () - cOS.degreeV + 1;

							// for surfaces, there are multiple X, Y, & Zs in each
							// row. This computes the number of points in each
							// column, though its not used here (multiply by 3
							// for the number of columns total, since there are
							// 3 columns for each control point.
							// nCols = cOS.knotsU.size () - cOS.degreeU + 1;
						}
						nRowsLeft = nRows - 1; // we've already read in the first row.
					}

					float tempCoord;
					float tempWeight;
					int whichCoord = 0; // 0 for X, 1 for Y, 2 for Z
					int index;   
					// process the line of control points:
					while (sstream >> tempCoord) {
						//tempPoint[whichCoord % 3] = tempCoord; 
						// put the value into X,Y or Z
						index=whichCoord%3;
						if(index==0)
							tempPoint.x=tempCoord;
						if(index==1)
							tempPoint.y=tempCoord;
						if(index==2)
							tempPoint.z=tempCoord;
						++whichCoord;
						if (whichCoord % 3 == 0) // once we've filled in a point, save it.
						{
							pointBuffer.push_back (tempPoint);
						}
					}
					getline (myfile, line);
					// only examine the line if it's not blank, and not a comment
					if (line.find ("//") == string::npos && line.find_first_not_of(" \t\v\r\n") != string::npos ) {
						// we'll need this for some parsing in spots. it's sometimes 
						// not used:
						sstream.clear ();
						sstream.str (line);
						//read weight value of each point
						while(sstream>>tempWeight)
						{
							weightBuffer.push_back(tempWeight);
						}
						if(weightBuffer.size()!=pointBuffer.size())
						{
							cout<<"number of weight and point are not match!"<<endl;
							exit(-1);
						}

						for(unsigned int i=0; i<pointBuffer.size();++i)
						{
							wPoint.p=pointBuffer[i];
							wPoint.w=weightBuffer[i];
							cOS.controlPoints.push_back(wPoint);
						}
						pointBuffer.clear();
						weightBuffer.clear();

					}

				}

				// this block of code decides if we're moving onto the next field,
				// or the next object, or just continuing to read in more control
				// points
				if ((cOS.geoType == 0||cOS.geoType==2) && currentField == 2) {
					// we're still parsing an object, but it's a curve and we
					// just finished reading the uknots. Skip over the vknots.
					currentField += 2;
				}
				else if (currentField == 4 && nRowsLeft > 0) {
					// we're in the middle of reading a block of control points.
					// don't skip to the next field - we still need to read in
					// some more rows of control points:
					--nRowsLeft;
				}
				else if (currentField == 4) {   // we're finished with this object.
					geoObjs.push_back (cOS);    // save onto our list of objects.
					cOS.reset ();               // reset the lists, etc.
					currentField = 0;           // reset back to the first field.
					nRowsLeft = -1;             // logic value - so that it's recomputed.
				}
				else    // we're still parsing an object, move to the next field
					++currentField;

			}
		}
		myfile.close();
		if (currentField != 0) {
			cout << "error: apparent stop in mid-parse of object" << endl;
			exit (-1);
		}
	}

	else {
		cout << "error: unable to open file"; 
		exit (-1);
	}

	return 0;
}


//
//
//
/*
   int main (int argc, const char * argv[])
   {

// place to put the objects descriptions that are parsed from the file:
vector< curveOrSurface > geoObjs;

// parse the file: in your code, you will want to read in the file from
// the argument. the geometric descriptions in the scene file are loaded
// into the vector specified in the second argument (geoObjs).
parse ("test_input.txt", geoObjs);



//
// print out some of data to check it's working properly:
//
for (int i = 0; i < geoObjs.size (); ++i ) {

cout << "object " << i << " is a ";

if (geoObjs[i].geoType) {
cout << "surface of degree " << geoObjs[i].degreeU << "x" << geoObjs[i].degreeV << endl;
cout << " knots in u: " << geoObjs[i].knotsU.size () << endl;
cout << " knots in v: " << geoObjs[i].knotsV.size () << endl;
}
else {
cout << "curve of degree " << geoObjs[i].degreeU << endl;
cout << " knots in u: " << geoObjs[i].knotsU.size () << endl;
}

cout << " control points: " << geoObjs[i].controlPoints.size () << endl;

// print out all the control points:
for (int k = 0; k < geoObjs[i].controlPoints.size (); ++k) {

geoObjs[i].controlPoints[k].display();
// cout << "   " << geoObjs[i].controlPoints[k][0] << " " <<
//geoObjs[i].controlPoints[k][1] << " " <<
//geoObjs[i].controlPoints[k][2] << endl;
}

}


// we could do other things here, maybe check that the number of control
// points matches the degree & # of knots in the curve/surface.
//

return 0;

}
*/
#endif
