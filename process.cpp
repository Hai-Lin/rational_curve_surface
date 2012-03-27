#include"process.h"
int getI(float u,const curveOrSurface& cOS, int rowOrColumn) //return i for row or column 0 for row, 1 for column
{
	int i;
	if (rowOrColumn==0) //this is for row
	{
		if(u==cOS.knotsU[cOS.degreeU-1])
			return cOS.degreeU-1;
		for(int p=cOS.degreeU-1; p<(int)cOS.knotsU.size();++p)
		{
			if(cOS.knotsU[p]<u)
				continue;
			else
			{
				i=p-1;
				break;
			}
		}
	}
	else
	{
		if(u==cOS.knotsV[cOS.degreeV-1])
			return cOS.degreeV-1;
		for(int p=cOS.degreeV-1;p<(int)cOS.knotsV.size();++p)
		{
			if(cOS.knotsV[p]<u)
				continue;
			else
			{
				i=p-1;
				break;
			}
		}
	}
	return i;
}

PointNormal getCurvePT(float u, const curveOrSurface& cOS )
{
	float t, w;
	WVector3D tempWCP;
	Vector3D point, wPoint1, wPoint2;
	PointNormal pn;
	vector<WVector3D> tempcp;
	int i=getI(u,cOS,0);
	for(int j=i-cOS.degreeU+1;j<=i+1;++j)
		tempcp.push_back(cOS.controlPoints[j]);
	for(int k=1; k<=cOS.degreeU;++k)  //for each loop, update the tempcp
	{
		for(int z=0;z<cOS.degreeU-k+1;++z)
		{
			if(z==cOS.degreeU-k)  //calaculate tangent vector
			{
				Vector3D tangentU=tempcp[1].p-tempcp[0].p;
				pn.tangentU=normalize(tangentU);
			}
			t=(u-cOS.knotsU[i-cOS.degreeU+k+z])/(cOS.knotsU[i+z+1]-cOS.knotsU[i-cOS.degreeU+k+z]);
			w=t*(tempcp[z+1].w-tempcp[z].w)+tempcp[z].w; 
			wPoint1=tempcp[z].w*tempcp[z].p;
			wPoint2=tempcp[z+1].w*tempcp[z+1].p;
			point=interpretate(wPoint1,wPoint2,t);
			point=point/w;
			tempWCP.w=w;
			tempWCP.p=point;
			tempcp[z]=tempWCP;
			//point.display();
		}
	}
	pn.point=tempcp[0].p;
	return pn;
}
/*return value of point and tangent evluated by de boor alg*/
PointNormal getPT(float u, const curveOrSurface& cOS, int row, int column, int points )
{
	float t,w;
	WVector3D tempWCP;
	PointNormal pn;
	//temp control points array, updata everytime in the loop
	vector<WVector3D> tempcp;
	Vector3D point, wPoint1, wPoint2;
	if(column==-1) //this is for row 
	{
		int i=getI(u,cOS,0);
		for(int j=i-cOS.degreeU+1;j<=i+1;++j)
			tempcp.push_back(cOS.controlPoints[points*j+row]);  //get control points[j,row]

		for(int k=1; k<=cOS.degreeU;++k)  //for each loop, update the tempcp
		{
			for(int z=0;z<cOS.degreeU-k+1;++z)
			{
				if(z==cOS.degreeU-k)  //calaculate tangent vector
				{
					Vector3D tangentU=tempcp[1].p-tempcp[0].p;
					pn.tangentU=normalize(tangentU);
				}
				t=(u-cOS.knotsU[i-cOS.degreeU+k+z])/(cOS.knotsU[i+z+1]-cOS.knotsU[i-cOS.degreeU+k+z]);
				w=t*(tempcp[z+1].w-tempcp[z].w)+tempcp[z].w; 
				wPoint1=tempcp[z].w*tempcp[z].p;
				wPoint2=tempcp[z+1].w*tempcp[z+1].p;
				point=interpretate(wPoint1,wPoint2,t);
				point=point/w;
				tempWCP.w=w;
				tempWCP.p=point;
				tempcp[z]=tempWCP;
			}
		}
		pn.point=tempcp[0].p;
		return pn;
	}
	else
	{
		int i=getI(u,cOS,1);
		for(int j=i-cOS.degreeV+1;j<=i+1;++j)
			tempcp.push_back(cOS.controlPoints[points*j+column]);  //get control point[j,column]
		for(int k=1;k<=cOS.degreeV;++k)
		{
			for(int z=0;z<cOS.degreeV-k+1;++z)
			{
				if(z==cOS.degreeV-k)
				{
					//	cout<<"calculation v tangent"<<endl;
					Vector3D tangentV=tempcp[1].p-tempcp[0].p;
					pn.tangentV=normalize(tangentV);
				}
				t=(u-cOS.knotsV[i-cOS.degreeV+k+z])/(cOS.knotsV[i+z+1]-cOS.knotsV[i-cOS.degreeV+k+z]);
				w=t*(tempcp[z+1].w-tempcp[z].w)+tempcp[z].w; 
				wPoint1=tempcp[z].w*tempcp[z].p;
				wPoint2=tempcp[z+1].w*tempcp[z+1].p;
				point=interpretate(wPoint1,wPoint2,t);
				point=point/w;
				tempWCP.w=w;
				tempWCP.p=point;
				tempcp[z]=tempWCP;
			}
		}
		pn.point=tempcp[0].p;
		return pn;
	}

}
WVector3D getU(float u, const curveOrSurface& cOS, int row, int column )
{
	float t, w;
	WVector3D tempWCP;
	Vector3D point, wPoint1, wPoint2;
	PointNormal pn;
	vector<WVector3D> tempcp;
	int pointsU=cOS.knotsU.size()-cOS.degreeU+1;  //number of control points in U
	if(column==-1) //this is for row 
	{
		int i=getI(u,cOS,0);
		for(int j=i-cOS.degreeU+1;j<=i+1;++j)
			tempcp.push_back(cOS.controlPoints[pointsU*row+j]);     //get control point [row, j]
		//for(int i=0;i<tempcp.size();++i)
		//	tempcp[i].display();

		for(int k=1; k<=cOS.degreeU;++k)  //for each loop, update the tempcp
		{
			for(int z=0;z<cOS.degreeU-k+1;++z)
			{
				t=(u-cOS.knotsU[i-cOS.degreeU+k+z])/(cOS.knotsU[i+z+1]-cOS.knotsU[i-cOS.degreeU+k+z]);
				w=t*(tempcp[z+1].w-tempcp[z].w)+tempcp[z].w; 
				wPoint1=tempcp[z].w*tempcp[z].p;
				wPoint2=tempcp[z+1].w*tempcp[z+1].p;
				point=interpretate(wPoint1,wPoint2,t);
				point=point/w;
				tempWCP.w=w;
				tempWCP.p=point;
				tempcp[z]=tempWCP;
			}
		}
		return tempcp[0];
	}
	else
	{
		int i=getI(u,cOS,1);
		//for(int j=i+1;j>=i-cOS.degreeV+1;--j)  //in reverse order to botton up
		for(int j=i-cOS.degreeV+1;j<=i+1;++j)
			tempcp.push_back(cOS.controlPoints[pointsU*j+column]);  //get control point[j,column]
		//	tempcp.push_back(cOS.controlPoints[pointsV*column+j]);
		for(int k=1;k<=cOS.degreeV;++k)
		{
			for(int z=0;z<cOS.degreeV-k+1;++z)
			{
				t=(u-cOS.knotsV[i-cOS.degreeV+k+z])/(cOS.knotsV[i+z+1]-cOS.knotsV[i-cOS.degreeV+k+z]);
				w=t*(tempcp[z+1].w-tempcp[z].w)+tempcp[z].w; 
				wPoint1=tempcp[z].w*tempcp[z].p;
				wPoint2=tempcp[z+1].w*tempcp[z+1].p;
				point=interpretate(wPoint1,wPoint2,t);
				point=point/w;
				tempWCP.w=w;
				tempWCP.p=point;
				tempcp[z]=tempWCP;
			}
		}
		return tempcp[0];
	}

}
vector<PointNormal> getTrimPoints( const curveOrSurface& cOS)
{
	vector<PointNormal> trim;
	int points=5*cOS.controlPoints.size();
	float domain=cOS.knotsU[cOS.knotsU.size()-cOS.degreeU]-cOS.knotsU[cOS.degreeU-1];
	float step=domain/((float)points+1);
	float t=cOS.knotsU[cOS.degreeU-1];
	for(int i=0; i<=points;++i)
	{
		PointNormal temp=getCurvePT(t,cOS);
		trim.push_back(temp);
		t=t+step;
		//temp.displayPN();
	}
	//do the last point
	PointNormal last=getCurvePT((float)cOS.knotsU[cOS.knotsU.size()-cOS.degreeU], cOS);
	trim.push_back(last);
	return trim;
}
/*return a vector of points of the curve didvid by factor*/
vector<PointNormal> getCurvePoints(int factor, const curveOrSurface& cOS)
{
	vector<PointNormal> curve;
	int points=2*pow((double)3,(factor-1));
	float domain=cOS.knotsU[cOS.knotsU.size()-cOS.degreeU]-cOS.knotsU[cOS.degreeU-1];
	float step=domain/((float)points+1);
	float t=cOS.knotsU[cOS.degreeU-1];
	for(int i=0; i<=points;++i)
	{
		PointNormal temp=getCurvePT(t,cOS);
		curve.push_back(temp);
		t=t+step;
	}
	//do th`e last point
	PointNormal last=getCurvePT((float)cOS.knotsU[cOS.knotsU.size()-cOS.degreeU], cOS);
	curve.push_back(last);
	return curve;
}
//make the control points from buttom up(u->v)
vector<WVector3D> buttomUp(const curveOrSurface& cOS)
{
	int pointsU=cOS.knotsU.size()-cOS.degreeU+1;  //number of control points in U
	int pointsV=cOS.knotsV.size()-cOS.degreeV+1;  //number of control points in v
	vector<WVector3D> cp;
	for(int i=pointsV-1; i>=0; --i)
		for (int j=0; j<pointsU;++j)
			cp.push_back(cOS.controlPoints[i*pointsU+j]);
	return cp;
}

/*return a vector of points of the surface devided by factor*/
vector<PointNormal> getSurfacePoints(int factor, curveOrSurface cOS)
{
	vector<WVector3D> tempV;  //temp points after evluation V
	vector<WVector3D> tempU;  //temp points after evluation U
	vector<PointNormal> ptV; //point with V tengent
	vector<PointNormal> ptU; // point with U tengent
	vector<PointNormal> final;  //final set of point normal
	cOS.controlPoints=buttomUp(cOS);
	//vector<Vector3D> final;
	int pointsU=cOS.knotsU.size()-cOS.degreeU+1;  //number of control points in U
	int pointsV=cOS.knotsV.size()-cOS.degreeV+1;  //number of control points in v
	int points=2*pow((double)3,(factor-1));
	float domainU=cOS.knotsU[cOS.knotsU.size()-cOS.degreeU]-cOS.knotsU[cOS.degreeU-1];
	float domainV=cOS.knotsV[cOS.knotsV.size()-cOS.degreeV]-cOS.knotsV[cOS.degreeV-1];
	float stepU=domainU/((float)(points+1));
	float stepV=domainV/((float)(points+1));
	for(int i=0; i<pointsV;++i)
	{
		float t=cOS.knotsU[cOS.degreeU-1];
		for(int p=0;p<=points;++p)
			//below is totally stupid stuff, do not do this again!!!!!!
			//for(float t=cOS.knotsU[cOS.degreeU-1]+step;t<cOS.knotsU[cOS.knotsU.size()-cOS.degreeU];t=t+step)
		{
			WVector3D temp=getU(t,cOS,i,-1);
			tempU.push_back(temp);
			t=t+stepU;
			//		temp.display();
		}
		WVector3D last=getU((float)cOS.knotsU[cOS.knotsU.size()-cOS.degreeU], cOS, i,-1);
		tempU.push_back(last);
		//last.display();
	}
	for(int i=0; i<pointsU;++i)
	{
		float t=cOS.knotsV[cOS.degreeV-1];
		for(int p=0; p<=points;++p)
		{
			WVector3D temp=getU(t,cOS,-1,i);
			tempV.push_back(temp);
			t=t+stepV;
			//temp.display();
		}
		WVector3D last=getU((float)cOS.knotsV[cOS.knotsV.size()-cOS.degreeV], cOS, -1,i);
		tempV.push_back(last);
		//last.display();
	}

	curveOrSurface tempUS, tempVS;
	tempUS=cOS;
	tempVS=cOS;
	tempUS.controlPoints=tempU;
	tempVS.controlPoints=tempV;
	//for(int i=0; i<newcOS.controlPoints.size();++i)
	//	newcOS.controlPoints[i].display();
	//domain=cOS.knotsV[cOS.knotsV.size()-cOS.degreeV]-cOS.knotsV[cOS.degreeV-1];
	//step=domain/((float)points+1);
	for(int i=0;i<points+2;++i)
	{
		//for(float t=cOS.knotsV[cOS.degreeV-1]+step;t<cOS.knotsV[cOS.knotsV.size()-cOS.degreeV];t=t+step)
		float t=cOS.knotsV[cOS.degreeV-1];
		for(int p=0;p<=points;++p)
		{
			PointNormal temp=getPT(t,tempUS,-1,i, points+2);
			ptV.push_back(temp);
			t=t+stepV;
			//temp.point.display();
		}
		PointNormal last=getPT((float)cOS.knotsV[cOS.knotsV.size()-cOS.degreeV], tempUS, -1,i, points+2);
		ptV.push_back(last);
		//last.point.display();
		//cout<<"one loop end"<<endl;
	}
	for(int i=0; i<points+2;++i)
	{
		float t=cOS.knotsU[cOS.degreeU-1];
		for(int p=0; p<=points;++p)
		{
			PointNormal temp=getPT(t,tempVS,i,-1, points+2);
			ptU.push_back(temp);
			t=t+stepU;
		}
		PointNormal last=getPT((float)tempVS.knotsU[tempVS.knotsU.size()-tempVS.degreeU], tempVS, i,-1, points+2);
		ptU.push_back(last);
	}
	ptV=transpose(ptV, points+2,points+2);
	for(unsigned int i=0; i<ptV.size();++i)
	{
		if(ptV[i].point==ptU[i].point)
			cout<<"u and v are not matching"<<endl;
		else
		{
			if(isVecZero(ptU[i].tangentU))
			{
				//cout<<"the "<<i<<"th ptU.tangent is null"<<endl;
				//set normal to up and down 
				if(i/(points+1))
				{
					ptU[i].normal=Vector3D(0,-1,0);
				}
				else
				{
					ptU[i].normal=Vector3D(0,1,0);
				}
			}


			if(isVecZero(ptV[i].tangentV))
				cout<<"ptV.tangent is null"<<endl;
			ptU[i].tangentV=ptV[i].tangentV;
			ptU[i].getNormal();
			//ptU[i].displayPN();
		}
	}
	return ptU;
}

vector<PointNormal> transpose( vector<PointNormal> origional, int row, int column)
{
	vector<PointNormal> temp;
	for(int i=0; i<column;++i)
		for(int j=0; j<row;++j)
			temp.push_back(origional[j*column+i]);
	return temp;
}
vector<bool> getTrimSurface(int factor, vector<PointNormal> trimSample, const curveOrSurface & cOS )
{

	vector<bool> isInTrim;
	float u,v;
	int points=2*pow((double)3,(factor-1));
	float domainU=cOS.knotsU[cOS.knotsU.size()-cOS.degreeU]-cOS.knotsU[cOS.degreeU-1];
	float domainV=cOS.knotsV[cOS.knotsV.size()-cOS.degreeV]-cOS.knotsV[cOS.degreeV-1];
	float stepU=domainU/((float)(points+1));
	float stepV=domainV/((float)(points+1));
	for( int i=0; i<((points+2)*(points+2));++i)
	{
		u=(i%(points+2))*stepU;
		v=(i/(points+2))*stepV;
		Vector3D temp(u,0,v);
		isInTrim.push_back(isInTrimCurve(temp, trimSample));
	}
	return isInTrim;
}
bool isInTrimCurve(Vector3D point , vector<PointNormal> trimSample) //return true if the point is in the curve in uv space, false otherwise. 
{
	Vector3D nt,r;
	int n=theNearest(point, trimSample);
	nt=trimSample[n].tangentU;
	r=point-trimSample[n].point;
	Vector3D product=nt*r;
	if(product.y<0)
		return true;
	else
		return false;
}

//return the index of the nearest point
int theNearest(Vector3D point, vector<PointNormal> trimSample)
{
	float min=3.40282e+37, d;
	int index;
	for(unsigned int i=0; i<trimSample.size();++i)
	{
		d=distanceVec(point,trimSample[i].point);
		if(d<min)
		{
			min=distanceVec(point,trimSample[i].point);
			index=i;
		}
	}
	return index;
}


