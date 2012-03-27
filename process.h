
#ifndef _PROCESS_H_
#define _PROCESS_H_
#include"utility.h"
//functions 
int getI(float u,const curveOrSurface& cOS, int rowOrColumn); //return i for row or column 0 for row, 1 for column
PointNormal getCurvePT(float u, const curveOrSurface& cOS);
PointNormal getPT(float u, const curveOrSurface& cOS, int row, int column, int points);
WVector3D getU(float u, const curveOrSurface& cOS, int row, int column );
vector<PointNormal> getCurvePoints(int factor, const curveOrSurface& cOS);
vector<PointNormal> transpose( vector<PointNormal> origional, int row, int column);
vector<WVector3D> buttomUp(const curveOrSurface& cOS);
vector<PointNormal> getTrimPoints(const curveOrSurface& cOS);
vector<PointNormal> getSurfacePoints(int factor, curveOrSurface cOS);
vector<bool> getTrimSurface(int, vector<PointNormal> , const curveOrSurface & cOS);   //return a bool vector of point in the the trim curve or not
bool isInTrimCurve(Vector3D , vector<PointNormal>); //return true if the point is in the curve in uv space, false otherwise. 
int theNearest(Vector3D, vector<PointNormal>);
#endif
