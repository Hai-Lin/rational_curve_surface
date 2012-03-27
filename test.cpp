#include"process.h"

int main(int argc, char** argv)
{
	vector<curveOrSurface> geoObjs;

	char *s1=argv[1];
	parse(s1,geoObjs);
	curveOrSurface coo;
	coo.geoType=0;
	coo.degreeU=2;
	coo.knotsU.push_back(0);
	coo.knotsU.push_back(0);
	coo.knotsU.push_back(1);
	coo.knotsU.push_back(2);
	coo.knotsU.push_back(2);
	coo.controlPoints.push_back(Vector3D(-1,0,0));
	coo.controlPoints.push_back(Vector3D(0,1,0));
	coo.controlPoints.push_back(Vector3D(1,1,0));
	coo.controlPoints.push_back(Vector3D(1,2,0));
	//Vector3D result=getU(1.5,coo,0,0);

	//vector<Vector3D> tt=drawCurve(3,coo);	

   // vector<PointNormal> dd=getTrimPoints(curveOrSurface cOS);
	vector<PointNormal> dd;
	dd=getTrimPoints(geoObjs[0]);
	vector<bool> test;
	test=getTrimSurface(2,dd,geoObjs[1]);
	cout<<test.size()<<endl;
	for(int i=0; i<test.size();++i)
	{	if(test[i])
			cout<<"1"<<endl;
		else
			cout<<"0"<<endl;
	}
	return 0;
}

