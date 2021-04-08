#include <math.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>
#include <queue>
#include <list>
#include <windows.h>
#include <stdio.h>
#include "Eigen/Dense"
#include <fstream>
#include <time.h>

#define PI 3.14159265
#define RAYSAMOUNT 1081

using namespace Eigen;
using namespace std;

//===========sam.cpp=============
enum LandMarkType{CONVEX, CONCAVE};
struct Point
{
	double x;
	double y;
};
struct Line
{
	float A;
	float B;
	float C;
};
struct Segment
{

	bool IsSegment;
	Line Param;
	int StartIndex;
	int EndIndex;
	float Angle;
	double Start_x, Start_y;
	double End_x, End_y;
	void Angle2Radian() {
		Angle = Angle*PI / 180;
	}
	void Radian2Angle() {
		Angle = Angle * 180 / PI;
	}
};
struct LandMark {

	bool HaveMark;
	float Theta, Theta_W_B;
	Point O_B_W, O_W_B;
	int Serial;
};
class SAM
{
public:
	SAM();
	~SAM();

	bool   landmark_found;
	double result_B_W_x, result_B_W_y, result_B_W_theta;
	double result_W_B_x, result_W_B_y, result_W_B_theta;
	

	//method
	//void doSam(double *laserdata, double L1, double L2);
	void doSam(double *laserdata, double L1, double L2, int RaysAmount, double AngleIncrease, double angle_start, LandMarkType LMTYPE);

private:

	//int RAYSAMOUNT;
	int NUM_SCAN;
	int RAYSTART;
	double DEG_INCREASE;
	int GROUP;
	double DIST_THRESHOLD;
	double LASER_THRESHOLD;
	int SEG_THRESHOLD;
	double ANGLE_START;
	LandMark Mark;

	double EDGE1, EDGE2;
	double ANGLE_LANDMARK;
	LandMarkType LMtype;

	//method
	LandMark SamforIcp(double *ScanData, Point *PointList, vector<Segment> &Segs, int RaysAmount = 541, double AngleIncrease = 0.5, double AngleStart = -135.0);
	void DistanceFilter(Point *Points, int PointNum, vector<int> &Index, int &IndexLength);
	void FindSegment(Point *PointList, int PointLength, vector<Point> &ResultList, int ResultLength, vector<int> &SegIndex);
	void sam(Point *TempList, int TempLength, vector<Point> &ResultList);
	LandMark FindMark(vector<Segment> &Segs, int SegLength, Point *PointList);
	double norm(Point PointA, Point PointB);
	Line LineFit(Point PointA, Point PointB);
	double P2LDist(Point P, Line L);
	Line LSFit(Point *Points, vector<int> &Index, int i);
	float AngleBtwSeg(Segment Seg1, Segment Seg2);
	Point Intersect(Segment Seg1, Segment Seg2);

	template <class T>
	int length(T& Array) {
		return (sizeof(Array) / sizeof(Array[0]));
	}

	template <class T>
	void DeleteVector(vector<T> &v) {
		vector<T> Tempv;
		Tempv.swap(v);
	}

};
SAM::SAM() :
//初始化参数
NUM_SCAN(25),
RAYSTART(0),
DEG_INCREASE(0.25),
GROUP(50),
DIST_THRESHOLD(0.2),
LASER_THRESHOLD(0.03),
SEG_THRESHOLD(10),
ANGLE_START(-90),
ANGLE_LANDMARK(120)
{}
SAM::~SAM()
{

}
void SAM::doSam(double *laserdata, double L1, double L2, int RaysAmount, double AngleIncrease, double angle_start, LandMarkType LMTYPE)
{
	EDGE1 = L1;
	EDGE2 = L2;
	LMtype = LMTYPE;
	//int nrays = (sizeof(laserdata) / sizeof(laserdata[0]));
	//    sam for icp
	Point *PointList = new Point[RaysAmount];
	vector<Segment> Segs;
	
	Mark = SamforIcp(laserdata, PointList, Segs, RaysAmount, AngleIncrease, angle_start);
	delete[]PointList;

	if (Mark.HaveMark)
	{
		landmark_found = true;
		result_B_W_x = Mark.O_B_W.x;
		result_B_W_y = Mark.O_B_W.y;
		result_B_W_theta = Mark.Theta;
		result_W_B_x = Mark.O_W_B.x;
		result_W_B_y = Mark.O_W_B.y;
		result_W_B_theta = Mark.Theta_W_B;
	}
	else
	{
		landmark_found = false;
		result_B_W_x = 0;
		result_B_W_y = 0;
		result_B_W_theta = 0;
		result_W_B_x = 0;
		result_W_B_y = 0;
		result_W_B_theta = 0;
	}

	DeleteVector(Segs);
}
LandMark SAM::SamforIcp(double *ScanData, Point *PointList, vector<Segment> &Segs, int RaysAmount, double Angle_Increase, double Angle_Start) {
	//    read data and generate PointList
	//std::cout << RaysAmount << std::endl;
	double *Theta = new double[RaysAmount];
	for (int i = 0; i < RaysAmount; i++) {
		Theta[i] = (i*Angle_Increase + Angle_Start) / 180 * PI;

		PointList[i].x = cos(Theta[i])*(*(ScanData + i)) ;
		PointList[i].y = sin(Theta[i])*(*(ScanData + i)) ;
	}

	//    preprocessing, clustering according to d between points
	int DistIndexLength = 1;
	vector<int> DistIndex;
	DistIndex.push_back(0);
	
	DistanceFilter(PointList, RaysAmount, DistIndex, DistIndexLength);

	//    do sam, find SegIndex
	int Start, Last;
	vector<Point> ResultList; 
	for (int i = 0; i < (DistIndexLength - 1); i++) 
	{
		Start = DistIndex[i];
		Last = DistIndex[i + 1];
		Point *ScanPartial = new Point[Last - Start + 1];
		for (int j = Start; j <= Last; j++) 
		{
			*(ScanPartial + j - Start) = PointList[j];
	    }
		vector<Point> TempList;
		sam(ScanPartial, Last - Start + 1, TempList);
		ResultList.insert(ResultList.end(), TempList.begin(), TempList.end());
		DeleteVector(TempList);
		delete[]ScanPartial;
	}
	DeleteVector(DistIndex);

	//std::cout << "ResultList size" << ResultList.size() << std::endl;

	vector<int> SegmentIndex;
	SegmentIndex.push_back(0);
	//std::cout << "start FindSegment" << std::endl;
	FindSegment(PointList, RaysAmount, ResultList, ResultList.size(), SegmentIndex);
	//std::cout << "After FindSegment" << std::endl;
	DeleteVector(ResultList);
	
	//find segment
	for (int i = 0; i < (SegmentIndex.size() - 1); i++) {
		Segment tempseg;
		if (abs(SegmentIndex[i] - SegmentIndex[i + 1]) > SEG_THRESHOLD)
		{
			// least square fitting
			tempseg.IsSegment = true;
			tempseg.Param = LSFit(PointList, SegmentIndex, i);
			tempseg.StartIndex = SegmentIndex[i];
			tempseg.EndIndex = SegmentIndex[i + 1];
			tempseg.Start_x = PointList[SegmentIndex[i]].x;
			tempseg.Start_y = PointList[SegmentIndex[i]].y;
			tempseg.End_x = PointList[SegmentIndex[i + 1]].x;
			tempseg.End_y = PointList[SegmentIndex[i + 1]].y;
			if (tempseg.Param.B == 0)
			{
				tempseg.Angle = 90;
			}
			else {
				float k = -tempseg.Param.A / tempseg.Param.B;
				tempseg.Angle = atan(k);
				tempseg.Radian2Angle();
				//if(k < 0) {
				//    tempseg.Angle += 180;
				//}
			}
		}
		else {
			tempseg.IsSegment = false;
			tempseg.StartIndex = SegmentIndex[i];
			tempseg.EndIndex = SegmentIndex[i + 1];
			tempseg.Angle = 0xFFFFFFFF;
		}
		Segs.push_back(tempseg);
	}

	LandMark Mark;
	//std::cout << "start FindLandMark" << std::endl;
	Mark = FindMark(Segs, SegmentIndex.size() - 1, PointList);
	DeleteVector(SegmentIndex);
	if (Mark.HaveMark == false) {
		cout << "Cannot find landmark!" << endl;
	}
	else {


		double the = Mark.Theta / 180 * M_PI;
		Mark.O_W_B.x = -cos(the)*Mark.O_B_W.x - sin(the)*Mark.O_B_W.y;
		Mark.O_W_B.y = sin(the)*Mark.O_B_W.x - cos(the)*Mark.O_B_W.y;

		// Change Mark.Theta into the angle of O_W_B
		Mark.Theta_W_B = -Mark.Theta;

		//cout << "Find out landmark!" << endl;
		//cout << "Line" << Mark.Serial << " & " << "Line" << Mark.Serial + 1 << endl;
		//cout << "StartIndex: " << Segs[Mark.Serial].StartIndex << endl;
		//cout << "MiddleIndex" << Segs[Mark.Serial].EndIndex << endl;
		//cout << "EndIndex: " << Segs[Mark.Serial + 1].EndIndex << endl;

		//cout << "===============" << endl;
		//cout << "Theta_rad = " << the << endl;
		//cout << "O_B_W.x,y = " << Mark.O_B_W.x << '\t' << Mark.O_B_W.y << endl;
		//cout << "O_W_B.x,y = " << Mark.O_W_B.x << '\t' << Mark.O_W_B.y << endl;
		//cout << "===============" << endl;

	}


	delete[]Theta;
	return Mark;
}
void SAM::DistanceFilter(Point *Points, int PointNum, vector<int> &Index, int &IndexLength) {
	for (int i = 0; i < (PointNum - 1); i++) {
		if (norm(Points[i], Points[i + 1]) > DIST_THRESHOLD) {
			if (i == Index[IndexLength - 1]) {
				IndexLength++;
				Index.push_back(i + 1);
			}
			else {
				IndexLength += 2;
				Index.push_back(i);
				Index.push_back(i + 1);
			}
		}
	}
	IndexLength++;
	Index.push_back(PointNum - 1);
}
void SAM::sam(Point *TempList, int TempLength, vector<Point> &ResultList) {
	double DistMax = 0;
	double Dist;
	int Index = 0;
	int Last = TempLength;
	//    Find the point with the maximum distance
	for (int i = 1; i < (Last - 1); i++) {
		Dist = P2LDist(*(TempList + i), LineFit(*TempList, *(TempList + Last - 1)));
		if (Dist > DistMax) {
			Index = i;
			DistMax = Dist;
		}
	}
	//    If max distance is greater than epsilon, recursively simplify
	if (DistMax > LASER_THRESHOLD) {
		vector<Point> RecResult1;
		vector<Point> RecResult2;
		Point *TempList1 = new Point[Index + 1];
		Point *TempList2 = new Point[Last - Index];
		for (int i = 0; i <= Index; i++) {
			*(TempList1 + i) = *(TempList + i);
		}
		for (int i = Index; i < Last; i++) {
			*(TempList2 + i - Index) = *(TempList + i);
		}
		sam(TempList1, Index + 1, RecResult1);
		sam(TempList2, Last - Index, RecResult2);
		vector<Point>(RecResult1).swap(RecResult1);  //minimize the space use of vector
		vector<Point>(RecResult2).swap(RecResult2);
		ResultList.clear();
		ResultList.insert(ResultList.begin(), RecResult2.begin(), RecResult2.end());
		ResultList.insert(ResultList.begin(), RecResult1.begin(), RecResult1.end());
		vector<Point>(ResultList).swap(ResultList);
		DeleteVector(RecResult1);  //release the memory
		DeleteVector(RecResult2);
		delete[]TempList1;
		delete[]TempList2;
	}
	else {
		ResultList.clear();
		ResultList.reserve(2);
		ResultList.push_back(*TempList);
		ResultList.push_back(*(TempList + Last - 1));
	}
}
void SAM::FindSegment(Point *PointList, int PointLength, vector<Point> &ResultList, int ResultLength, vector<int> &SegIndex)
{
	//vector<double> Value_x;
	//vector<double> Value_y;
	vector<Point> Value;
	for (int i = 0; i < ((ResultLength)-1); i++) {
		if (ResultList[i].x - ResultList[i + 1].x == 0) {
			//Value_x.push_back(ResultList[i].x);
			//Value_y.push_back(ResultList[i].y);
			Value.push_back(ResultList[i]);
		}
	}
	//std::cout << "Value size = " << Value.size()<<std::endl;
	int v = 0;
	//std::cout << "PointLenth" << PointLength<<std::endl;
	for (int j = 0; j < Value.size(); j++)
	{
		for (int k = v; v < PointLength; k++)
		{
			if ((PointList + k)->x == Value[j].x && (PointList+k)->y == Value[j].y)
			{
				SegIndex.push_back(k);
				v = k + 1;
				break;
			}
		}
	}
	//std::cout << "after find segment" << std::endl;
	return;
}
LandMark SAM::FindMark(vector<Segment> &Segs, int SegLength, Point *PointList)
{
	//std::cout << "size of LINE = " << Segs.size()<<std::endl;
	LandMark Mark;
	vector<LandMark> MarkList;
	for (int i = 0; i < (SegLength - 1); i++)
	{
		if (Segs[i].IsSegment && Segs[i + 1].IsSegment)
		{
			if (abs(AngleBtwSeg(Segs[i], Segs[i + 1]) - (180 - ANGLE_LANDMARK)) < 10)
			{
				//cout << "angle ok, check length" << endl;
				int start1 = Segs[i].StartIndex;
				int start2 = Segs[i + 1].StartIndex;
				int end1 = Segs[i].EndIndex;
				int end2 = Segs[i + 1].EndIndex;
				double length1 = norm(PointList[start1 + 2], PointList[end1]);
				double length2 = norm(PointList[start2], PointList[end2 - 2]);
				double error1 = abs(length1 - EDGE1);
				double error2 = abs(length2 - EDGE2);

				/*				cout << "///////////////////////" << endl;
				cout << "line"<<i<<" length1 " << length1 << endl;
				cout << "line"<<i+1<<" length2 " << length2 << endl;
				cout << "error1 " << error1 << endl;
				cout << "EDGE1 " << EDGE1 << endl;
				cout << "error2 " << error2 << endl;
				cout << "///////////////////////" << endl;
				*/

				if (error1<0.2 && error2<0.2)
					////////////////////////////////////////////// if (length1> 0.3 && length1 < 1 && length2 > 0.3 && length2 < 1)
				{
					LandMark Mark_temp;
					Mark_temp.O_B_W = Intersect(Segs[i], Segs[i + 1]);
					double delta_angle = (180 - AngleBtwSeg(Segs[i], Segs[i + 1])) / 2;
					double theta_rad, theta_angle;
					double k;
					
					switch (LMtype)
					{
					case CONVEX:
						delta_angle = (180 - AngleBtwSeg(Segs[i], Segs[i + 1])) / 2;
						cout << "CONVEX" << endl;
						break;
					case CONCAVE:
						delta_angle = 90 + AngleBtwSeg(Segs[i], Segs[i + 1]) / 2;
						//cout << "CONCAVE" << endl;
						break;
					}
					if (Segs[i].Param.B != 0)
					{
						k = -Segs[i].Param.A / Segs[i].Param.B;
						theta_rad = atan(k);
						theta_angle = theta_rad*180.0 / M_PI;

						if (theta_angle >= 0 && Segs[i].Start_x < Segs[i].End_x)
							theta_angle = theta_angle - 180;
						else if (theta_angle<0 && Segs[i].Start_x < Segs[i].End_x)
							theta_angle = theta_angle + 180;
					}
					else
					{
						if (Segs[i].Start_y > Segs[i].End_y)
							theta_angle = 90;
						else
							theta_angle = -90;
					}

					//cout << "++++++++++FindMark++++++++++" << endl;
					//cout << "i = " << i << endl;
					//cout << "Segs[i].StartIndex = " << Segs[i].StartIndex << endl;
					//cout << "Segs[i].EndIndex = " << Segs[i].EndIndex << endl;
					//cout << "Segs[i].Start_x,y =" << Segs[i].Start_x << '\t' << Segs[i].Start_y << endl;
					//cout << "Segs[i].End_x,y =" << Segs[i].End_x << '\t' << Segs[i].End_y << endl;
					//cout << "Segs[i].Param =" << Segs[i].Param.A << '\t' << Segs[i].Param.B << '\t' << Segs[i].Param.C << endl;
					//cout << "Segs[i].Angle =" << Segs[i].Angle << endl;
					//cout << "delta_angle=" << delta_angle << endl;
					//cout << "theta_angle=" << theta_angle << endl;
					//cout << "k = " << k << endl;
					//cout << "======================" << endl;
					Mark_temp.Theta = theta_angle + delta_angle;
					Mark_temp.Serial = i;
					Mark_temp.HaveMark = true;
					MarkList.push_back(Mark_temp);
				}
			}
		}
	}
	
	int MarkListSize = MarkList.size();
	//cout << "+++++++++++FindMark+++++++++" << endl;
	//cout << "MarkListSize  " << MarkListSize << endl;
	if (!MarkListSize)
	{
		Mark.HaveMark = false;
		return Mark;
	}
	else
	{
		//find out true landmark
		//Method in use: return the mark with smallest abs(theta); 
		//double smallest_y = fabs(MarkList[0].O_B_W.y);
		double smallest_dist = MarkList[0].O_B_W.y*MarkList[0].O_B_W.y + MarkList[0].O_B_W.x*MarkList[0].O_B_W.x;
		int smallest_index = 0;
		double dist = 0;
		cout << "MarkListSize:"<<MarkListSize << endl;
		for (int i = 0; i < MarkListSize; i++)
		{
			dist = MarkList[i].O_B_W.y*MarkList[i].O_B_W.y + MarkList[i].O_B_W.x*MarkList[i].O_B_W.x;
			if (dist < smallest_dist)
			{
				smallest_index = i;
				smallest_dist = dist;
			}
		} 

		return MarkList[smallest_index];
	}
}
double SAM::norm(Point PointA, Point PointB) {
	return sqrt((PointA.x - PointB.x)*(PointA.x - PointB.x) + (PointA.y - PointB.y)*(PointA.y - PointB.y));
}
Line SAM::LineFit(Point PointA, Point PointB) {
	double X = PointB.x - PointA.x;
	double Y = PointB.y - PointA.y;
	Line L;
	if (X == 0) {
		L.A = 1;
		L.B = 0;
		L.C = -PointA.x;
	}
	else {
		L.A = Y;
		L.B = -X;
		L.C = PointA.y*X - PointA.x*Y;
	}
	return L;
}
double SAM::P2LDist(Point P, Line L) {
	return (abs(L.A*P.x + L.B*P.y + L.C) / sqrt(L.A*L.A + L.B*L.B));
}
Line SAM::LSFit(Point *Points, vector<int> &Index, int i) {
	Line L;
	L.A = 0;
	L.B = 0;
	L.C = 0;
	float SumX = 0, SumY = 0, SumXY = 0, SumX2 = 0, b;
	int N = Index[i + 1] - Index[i] + 1;
	for (int j = Index[i]; j <= Index[i + 1]; j++) {
		SumX += Points[j].x;
		SumY += Points[j].y;
		SumXY += Points[j].x*Points[j].y;
		SumX2 += Points[j].x*Points[j].x;
	}
	b = (SumXY / N - SumX*SumY / (N*N)) / (SumX2 / N - (SumX / N)*(SumX / N));
	if (abs(b) >= 5000) {
		L.A = 1;
		L.B = 0;
		L.C = -SumX / N;
	}
	else {
		L.A = b;
		L.B = -1;
		L.C = SumY / N - b*SumX / N;
	}
	return L;
}
float SAM::AngleBtwSeg(Segment Seg1, Segment Seg2) {
	float A1 = Seg1.Param.A;
	float B1 = Seg1.Param.B;
	float A2 = Seg2.Param.A;
	float B2 = Seg2.Param.B;
	float CosA = abs(A1*A2 + B1*B2) / (sqrt(A1*A1 + B1*B1)) / (sqrt(A2*A2 + B2*B2));
	float AlphaRad = acos(CosA);
	float Alpha = AlphaRad * 180 / M_PI;
	return Alpha;
}
Point SAM::Intersect(Segment Seg1, Segment Seg2) {
	MatrixXd M1(2, 2);
	MatrixXd M2(2, 1);
	M1(0, 0) = Seg1.Param.A;
	M1(0, 1) = Seg1.Param.B;
	M1(1, 0) = Seg2.Param.A;
	M1(1, 1) = Seg2.Param.B;
	M2(0, 0) = -Seg1.Param.C;
	M2(1, 0) = -Seg2.Param.C;
	M2 = M1.inverse()*M2;
	Point P;
	P.x = M2(0, 0);
	P.y = M2(1, 0);
	return P;
}
//=================================


class BatteryCharge
{
protected:
	middleware::Node *nh;
    middleware::RPC rpc;
	middleware::RPC rpc_server;
	middleware::ConfigFile cfg;
private:
	struct _Rpc_Thread:public BThread
    {
        BatteryCharge* handle;
        _Rpc_Thread(BatteryCharge* p)
        {
            handle = p;
        }
        ~_Rpc_Thread()
        {
            kill();
        }
        void run()
        {
			static int count = 0;
			try{
                while(true)
                {
					count++;
                    handle->rpc.run();
					_sleep(1);
					
                }
			}
			catch (const std::exception& e)
			{
				std::cerr << count<<":rpc communication error: " << e.what() << std::endl;
			}
        }
    }rpc_thread;

	struct _Rpc2_Thread:public BThread
	{
		BatteryCharge* handle;
		_Rpc2_Thread(BatteryCharge* p)
		{
			handle = p;
		}
		~_Rpc2_Thread()
		{
			kill();
		}
		void run()
		{
			static int count = 0;
			try{
				while (true)
				{
					count++;
					handle->rpc_server.run();
					_sleep(1);

				}
			}
			catch (const std::exception& e)
			{
				std::cerr << count << ":rpc communication error: " << e.what() << std::endl;
			}
		}
	}rpc2_thread;
	

    struct _Node_Thread:public BThread
    {
        BatteryCharge* handle;
        _Node_Thread(BatteryCharge* p)
        {
            handle = p;
        }
        ~_Node_Thread()
        {
            kill();
        }
        void run()
        {
			static int count = 0;
			try{
                while(true)
                {
                    handle->nh->run();
					count++;
					_sleep(1); 
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<count<<":nh communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_thread;

	struct _Thread1 :public BThread
	{
			BatteryCharge* handle;
			BCond cond;
			BMutex _lock;

			_Thread1(BatteryCharge* p){ handle = p; }
			~_Thread1(){ kill(); }
			void run()
			{
				try
				{
					while (true)
					{
						stdmsg::SomeFlag tmp1, tmp2;
						tmp1 = handle->rpc.call<stdmsg::SomeFlag, stdmsg::SomeFlag>("get_docking_state", tmp2);
						_lock.lock();
						handle->docking_state.set_emergency_stop_flag(tmp1.emergency_stop_flag());
						handle->docking_state_int = handle->docking_state.emergency_stop_flag();
						_lock.unlock();
						_sleep(5);
					}
				}
				catch (const std::exception& e)
				{
					std::cerr <<  "Charging_state_Manager_error: " << e.what() << std::endl;
				}
			}

	}Charging_State_Manager;

		//fyf:2016-10-3 
	//====================
	int RECEIVE_CHARGING_REQUEST;
	int TERMINATE_CHARGE;
	stdmsg::Pose goal_pose1;
	stdmsg::Pose goal_pose3;
	stdmsg::Pose  goal_pose2;
	stdmsg::MoveMode move_mode;
	stdmsg::SomeFlag docking_state;
	volatile int docking_state_int;
	std::string global_plan_topic;
	double _angle_start_in_sam;
	int _is_goal_sent;
	int _goal_reached;
	float _landmark_L1;
	float _landmark_L2;
	double _angle_landmark;
	SAM sam;
	LandMarkType _landmark_type;
	int _state;
	double _safe_dis;
	double _final_y_compensate;
	double _py, _pw;// pid parameter for go_straight_to_dock()
	clock_t start_time, end_time;
	std::string str_logfile;
	//ofstream log_file;
	//===================
	std::string correct_scan_topic;
    double thresh_max;
    double thresh_min;
	double thresh_degree;
    int Start_index_;
    int End_index_;
	int Start_index_Begin;
    int End_index_Begin;
    double ANGLE_MIN_;
    double ANGLE_INCREMENT_;
    bool initial_;
    double result_x;
    double result_y;
    bool object_found;
    double safe_distance;
    double robot_size;
	int mark2 = 1;
	int count2 = 0;
	std::ofstream SaveFile;
	std::string save_file_name;
	bool _if_save_file;
	PARAMETER(double, stop_delta_distance);
	PARAMETER(double, stop_delta_theta);
	PARAMETER(double, nav_v);
	PARAMETER(double, nav_w);
public:
    BatteryCharge(int argc, char** argv) :
	    rpc_thread(this),
		rpc2_thread(this),
		nh_thread(this),
		Charging_State_Manager(this),
        //thresh_min(0.06), 
        //thresh_max(2.0),
        Start_index_(0),
        End_index_(0),
        //ANGLE_MIN_(-135.0),
        //ANGLE_INCREMENT_(0.5),
        initial_(true),
        result_x(0.0),
        result_y(0.0),
        object_found(false),
		_goal_reached(0),
		_is_goal_sent(0),
		_if_save_file(false),
		//_state(0), // default state: idle
		save_file_name(" "),
		//RECEIVE_CHARGING_REQUEST(-1), # my demo, initial value set to -1
		RECEIVE_CHARGING_REQUEST(-1), 
		TERMINATE_CHARGE(0)
    {
		
		if (argc == 2)
		{
			save_file_name = argv[1];
			//cout << save_file_name << endl;
			_if_save_file = true;
			//return;
			
		}
		
		std::string cfgfile("navigator.ini");
		for (int i = 0; i < argc - 1; i++)
			if( strcmp(argv[i],"-cfg") == 0 )
				cfgfile = argv[i+1];
		cfg.read(cfgfile);

		#undef helper__
        #define helper__(type, x, _default) set_##x ( type(cfg.value("local_plan", #x, _default) ) );\
        std::cout<<"[BatteryCharge] "#x<<"\t = ("<<#type<<") "<<x()<<std::endl;
        helper__(double, stop_delta_distance, 0.1);//m
		helper__(double, stop_delta_theta, 10);//°
		helper__(double, nav_v, 0.3);//m/s
		helper__(double, nav_w, 0.25);//rad/s

		#undef helper__
        #define helper__(type, x, _default) x = type(cfg.value("follow", #x, _default));\
        std::cout<<"[BatteryCharge] "#x<<"\t = ("<<#type<<") "<<x<<std::endl;
		helper__(double,thresh_min,0.06);
		helper__(double,thresh_max,1.0);
		helper__(double,safe_distance,0.25);
		helper__(double,thresh_degree,40);

        robot_size = double(cfg.value("laser_pose", "laser_x", "0.0"));
		//fyf:2016-10-3
		
		_state = int(cfg.value("auto_recharge", "initial_state", "0"));
		_angle_start_in_sam = double(cfg.value("sam", "start_angle", "-135.0"));
		_landmark_L1 = double(cfg.value("sam", "L1", "0.4"));
		_landmark_L2 = double(cfg.value("sam", "L2", "0.4"));
		_angle_landmark = double(cfg.value("sam", "angle_landmark", "120"));
		int temp_lmtype = int(cfg.value("sam", "LandMarkType", "0"));
		cout << "LandmarkType = " << temp_lmtype << endl;
		_landmark_type = (LandMarkType)temp_lmtype;
		goal_pose1.mutable_position()->set_x(double(cfg.value("auto_recharge", "goal_x", "25.00")));
		goal_pose1.mutable_position()->set_y(double(cfg.value("auto_recharge", "goal_y", "25.00")));
		goal_pose1.mutable_orentation()->set_yaw(double(cfg.value("auto_recharge", "goal_yaw", "0.00")));
		goal_pose3.mutable_position()->set_x(double(cfg.value("auto_recharge", "goal_x3", "25.00")));
		goal_pose3.mutable_position()->set_y(double(cfg.value("auto_recharge", "goal_y3", "25.00")));
		goal_pose3.mutable_orentation()->set_yaw(double(cfg.value("auto_recharge", "goal_yaw3", "0.00")));
		
		goal_pose2.mutable_position()->set_x(double(cfg.value("auto_recharge", "goal2_x", "25.00")));
		goal_pose2.mutable_position()->set_y(double(cfg.value("auto_recharge", "goal2_y", "25.00")));
		goal_pose2.mutable_orentation()->set_yaw(double(cfg.value("auto_recharge", "goal2_yaw", "25.00")));

		str_logfile = cfg.value("auto_recharge", "log_file", "log_docking_duration.txt");
		//node set and connect, copy from robot.cpp
		std::string tmp = cfg.value("node", "bind_batterycharge", "tcp://127.0.0.1:9005");
		std::cout<<"[node] bind = "<< tmp <<std::endl;
		nh = new middleware::Node(tmp);
		
		tmp = std::string(cfg.value("node", "connect_batterycharge", "tcp://127.0.0.1:9001;tcp://127.0.0.1:9002;tcp://127.0.0.1:9100"));
		std::cerr<<"[node] connect = " ;
		while( tmp != "" )
		{
			int pos = tmp.find(";");
			std::cerr<<tmp.substr(0, pos)<<"; ";
			nh->connect(tmp.substr(0, pos));
			if (pos != std::string::npos)
				tmp = tmp.substr(pos + 1 );
			else
				tmp = "";
		}
		std::cerr<<std::endl;

		
		tmp = std::string(cfg.value("rpc", "address_batterycharge", "tcp://127.0.0.1:8998"));
		std::cout << "[rpc] address_batterycharge: " << tmp << std::endl;
		try{
			rpc_server.bind(tmp);
			rpc_server.set("set_state", &BatteryCharge::set_state, this);
			rpc_server.connect(tmp);
		}
		catch (std::exception &e)
		{
			std::cerr << e.what() << std::endl;
			exit(-1);
		}
		
		tmp = std::string( cfg.value("rpc", "address", "tcp://127.0.0.1:9000") );
        rpc.connect(tmp);
        std::cerr<<"[RPC] connect = "<<tmp<<";"<<std::endl;

        correct_scan_topic  = std::string( cfg.value("topic", "scan", "scan") );
        nh->subscrible(correct_scan_topic, &BatteryCharge::update, this);

		//2016-10-3: fyf  从navigator订阅goal_reached.
		global_plan_topic = std::string(cfg.value("topic", "global_plan", "global_plan"));
		nh->subscrible(global_plan_topic, &BatteryCharge::update_goal_reached, this);

		//2016-10-25:fyf 工博会 
		std::string go_mark_topic = "go_mark";
		nh->subscrible(go_mark_topic, &BatteryCharge::update_go_mark, this);
		
		_safe_dis = double(cfg.value("auto_recharge", "final_safe_dis", "0.25"));
		_final_y_compensate = double(cfg.value("auto_recharge", "final_y_compensate", "0.0"));
		_py = double(cfg.value("auto_recharge", "py", "0.0"));
		_pw = double(cfg.value("auto_recharge", "pw", "0.0"));

		this->rpc_thread.start();
		this->rpc2_thread.start();
		this->nh_thread.start();
		//this->Charging_State_Manager.start();
		//_sleep(1000);
		//stdmsg::String str;
		//str.set_str("mark_reach");
		//cout << "mark_reach_sent" << endl;
		//this->nh->publish("mark_reach", str);

    }


	stdmsg::Data set_state(const stdmsg::Data &ReCvState)
	{
		stdmsg::Data ret;
		stdmsg::Velocity cmd;
		//_state = ReCvState.seq();
		ret.set_seq(ReCvState.seq());
		RECEIVE_CHARGING_REQUEST = ReCvState.seq();
		
		if (RECEIVE_CHARGING_REQUEST) //RECEIVE_CHARGING_REQUEST = 1
		{
			_state = 1;
		}

		else // RECEIVE_CHARGING_REQUEST = 0
		{
			if (_state == 6)
			{
				_state = 8;
			}
			else
			{
				_state = 1;
				cmd.set_v(0);
				cmd.set_w(0);
				this->nh->publish("set_cmd_batterycharge", cmd);
				set_move_mode(0);
				//RECEIVE_CHARGING_REQUEST = -1;
			}
		}
		

		return ret;
	}
	
	//crti:2016-08-18,检测前方一定范围内有没有障碍物
	//和localizer.cpp里的一样，参数不同
	int find_obstacle_forward(const stdmsg::Laser_Scan & rscan)
	{
		int cnt = 0;
		double min_distance,max_distance;//距离范围，单位米
		int min_laser,max_laser;

		/*
		//crti:2016-08-18,换成直接用BatteryCharge里的public变量，cal_object_position里面会初始化这些变量，然后set_vel才调用这个函数
		min_distance = 0.4;
		max_distance = 0.9;
		double angle = 60.0;//角度范围，60.0表示-30度到+30度
		//crti:2016-08-16,原来是按照1080线算的，现在变成动态的了
		min_laser = (int)((rscan.ranges().size()-1)/2+1 - angle*0.5 * 4);
	    if (min_laser < 0){ min_laser = 0; std::cout << "ob,out range,<" << std::endl; }
	    max_laser = (int)((rscan.ranges().size()-1)/2+1 + angle*0.5 * 4);
	    if (max_laser > rscan.ranges().size()-1){ max_laser = rscan.ranges().size()-1; std::cout << "ob,out range,>" << std::endl; }
		*/
		min_distance = this->thresh_min;
		max_distance = this->safe_distance;
		min_laser = this->Start_index_Begin;
		max_laser = this->End_index_Begin;
		for(int i=min_laser;i<max_laser;++i)
		{
			double r = rscan.ranges(i);
			if(r>min_distance && r<max_distance)
			{
				++cnt;
			}
		}

		if(cnt>24)//凭感觉给的，60度一共240个点，50个点在范围内就认为有物体
		{
			//std::cout<<"1";
			return 1;
		}
		else
		{
			//std::cout<<"0";
			return 0;
		}
	}

	stdmsg::Pose cal_object_position_W_B(const stdmsg::Laser_Scan & scan)
	{
		stdmsg::Pose pos_;

		int size = scan.ranges_size();
		//uint: rad
		double AngleIncrease = const_cast<stdmsg::Laser_Scan&>(scan).mutable_config()->angle_increment();
		//cout << "Angle_increase  " << AngleIncrease << endl;
	
		AngleIncrease = AngleIncrease / M_PI * 180; 

		vector<double> laserdata;
		for (int i = 0; i < size; i++)
		{
			laserdata.push_back(scan.ranges(i));
			//_sleep(100);
		}

		//double L1 = 400 / 1000.0;
		//double L2 = 400 / 1000.0;
		//cout << _landmark_L1 <<" "<< _landmark_L2 << " "<<size << " " <<AngleIncrease/M_PI*180<<" "<<_angle_start_in_sam << endl;
		sam.doSam(&laserdata[0], _landmark_L1, _landmark_L2, size, AngleIncrease, _angle_start_in_sam, _landmark_type);

		pos_.mutable_position()->set_x(sam.result_W_B_x);
		pos_.mutable_position()->set_y(sam.result_W_B_y);
		pos_.mutable_orentation()->set_yaw(sam.result_W_B_theta);

		//pos_.mutable_position()->set_x(0);
		//pos_.mutable_position()->set_y(0);
		//pos_.mutable_orentation()->set_yaw(0);

		return pos_;
	}

    stdmsg::Pose cal_object_position_B_W(const stdmsg::Laser_Scan & scan)
    {
		stdmsg::Pose pos_;

		int size = scan.ranges_size();
		//uint: rad
		double AngleIncrease = const_cast<stdmsg::Laser_Scan&>(scan).mutable_config()->angle_increment();
		//cout << "Angle_increase  " << AngleIncrease << endl;
		AngleIncrease = AngleIncrease / M_PI * 180; // in degrees
		
		vector<double> laserdata;
		for (int i = 0; i < size; i++)
		{
			laserdata.push_back(scan.ranges(i));
			//_sleep(100);
		}

		//double L1 = 400 / 1000.0;
		//double L2 = 400 / 1000.0;
		//cout << _landmark_L1 <<" "<< _landmark_L2 << " "<<size << " " <<AngleIncrease/M_PI*180<<" "<<_angle_start_in_sam << endl;
		sam.doSam(&laserdata[0], _landmark_L1, _landmark_L2, size, AngleIncrease, _angle_start_in_sam, _landmark_type);

		pos_.mutable_position()->set_x(sam.result_B_W_x);
		pos_.mutable_position()->set_y(sam.result_B_W_y);
		pos_.mutable_orentation()->set_yaw(sam.result_B_W_theta);

		//pos_.mutable_position()->set_x(0);
		//pos_.mutable_position()->set_y(0);
		//pos_.mutable_orentation()->set_yaw(0);

		return pos_;
    }

    int set_goal(const stdmsg::Pose& pos)
    {
		stdmsg::Pose goal_pos;

		goal_pos.mutable_position()->set_x(pos.position().x());
		goal_pos.mutable_position()->set_y(pos.position().y());
		goal_pos.mutable_orentation()->set_yaw(pos.orentation().yaw());

		set_move_mode(1);

		rpc.call<stdmsg::Pose, stdmsg::Pose>("set_goal", goal_pos);
		_sleep(50);
		rpc.call<stdmsg::Pose, stdmsg::Pose>("set_goal", goal_pos);

		_is_goal_sent = 1;
		 
		return 0;
    }

	inline void set_move_mode(int mm){
		move_mode.set_move_mode(mm);
		rpc.call<stdmsg::MoveMode, stdmsg::MoveMode>("set_movemode", move_mode);

	}

	void set_vel(const stdmsg::Pose& pos, const stdmsg::Laser_Scan & scan)
	{
		stdmsg::Velocity cmd;
		double x = pos.position().x();
		double y = pos.position().y();
		double theta = pos.orentation().yaw();
		double v = 0.0, w = 0.0;
		double stop_delta_distance = 0.23;
		double error_theta, error_distance;
		double p1, p2;
		int sign = 0;

		if (fabs(theta) > 15)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		if (x == 0 && y == 0 && theta == 0)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		//if(pos.orentation().yaw()<0)

		else
		{
			error_theta = atan(y/x);
			error_distance = sqrt(x*x + y*y) - safe_distance+0.1;

			
			while(error_theta> 3.141592653) error_theta -= 3.141592653*2;
			while(error_theta<-3.141592653) error_theta += 3.141592653*2;

			if (error_distance > 0.5 && error_distance < 5)
			{
				v = nav_v()*error_distance / 3;
				//v = 0.08;
			}
			else if (error_distance > 0 && error_distance < 1)
			{
				v = nav_v()*error_distance / 3;
				//v = 0.05;
			}
			else if (error_distance < 0){
				v = 0.0;
			}
			
			p1 = 0.3;
			p2 = 0.07;
			w = p1*y + p2*theta*M_PI/180;
			//
			//w = error_theta * 0.5;
			//if (w > nav_w())
			//	w = nav_w();
			//if (w < -nav_w())
			//	w = -nav_w();

		   // if (fabs(error_theta) > stop_delta_theta()*3.141592653 / 180)
			
			if (0)  //rotation only
			{
				error_theta = pos.orentation().yaw();
				v = 0.0;
				w = error_theta*M_PI / 180 * 1;
				if (w > nav_w())
					w = nav_w();
				if (w < -nav_w())
					w = -nav_w();
			}
			
		}

		if (v > 0.1){ v = 0.1; }
		if (fabs(w) > 0.5)
		{
		    sign = (w > 0) ? 1 : -1;
			w = 0.5*sign;
		
		}
		
		cmd.set_v(v);
        cmd.set_w(w);
		//std::cout << "err_d,err_a = " << error_distance << "," << error_theta << std::endl;
		//std::cout<<"velocity"<<" v="<<v<<" w="<<w<<std::endl;
		this->nh->publish("set_cmd_batterycharge",cmd);
		return;
	}

	void reset()
	{
		initial_ = true;
	}

	void update(const stdmsg::Laser_Scan & scan )
    {
		
		//cout << "REC= " << RECEIVE_CHARGING_REQUEST <<"  _state="<<_state<< endl;
		stdmsg::Pose pos;
		if (_if_save_file) { _state = 7; }
		stdmsg::Velocity cmd_temp;
		double v = 0.5, w = 0.0;
		static int i = 0;
		i++;
		if (i == 1)
		{
			start_time = clock();
			cout << "move_mode_sent" << endl;
			set_move_mode(4);
			return;
		}
		switch (_state){
		case 0: // idle
			//cout << scan.ranges_size() << endl;
			//stdmsg::Laser_Scan scan_temp = scan;
			//move_mode.set_move_mode(3);
			//rpc.call<stdmsg::MoveMode, stdmsg::MoveMode>("dataserver_movemode", move_mode);
			//_sleep(100);
            //
			//cout << "State: Idle" << endl;
		    //cmd_temp.set_v(0);
			//cmd_temp.set_w(0.07);
			//this->nh->publish("set_cmd_batterycharge", cmd_temp);
			get_docking_state();
			//std::cout << "Docking_state=" << docking_state_int << endl;
			pos = this->cal_object_position_B_W(scan);
			std::cout << "State:0 " << "(" << sam.landmark_found << "," << pos.position().x() << "," << pos.position().y() << "," << pos.orentation().yaw()<< " deg) " << endl;;
			//cout << "cmd_sent" << endl;
			break;
			
		case 1:
						
			//go to the pre-set position in front of the landmark, using SLAM
			std::cout << "State1: ";
			do_case1(scan); break;
			//do_case1_demo_0108(scan); break;

		case 2:
			pos = this->cal_object_position_W_B(scan);
			//std::cout <<"State:2 "<< "(" << sam.landmark_found << "," << pos.position().x() << "," << pos.position().y() << "," << pos.orentation().yaw() << ")" << std::endl;
			do_case2(pos,scan); 
			break;
		case 3:
			pos = this->cal_object_position_B_W(scan);
			std::cout <<"State:3 "<< "(" << sam.landmark_found << "," << pos.position().x() << "," << pos.position().y() << "," << pos.orentation().yaw() << ")" << std::endl;
			do_case3(pos, scan);
			break;

		case 4:
			pos = this->cal_object_position_B_W(scan);
			std::cout << "State:4"<<"(" << sam.landmark_found << "," << pos.position().x() << "," << pos.position().y() << "," << pos.orentation().yaw() << ")" << std::endl;
			do_case4(pos,scan); 
			break;
		case 5:
			// finish charging, because battery is full of something else
			// robot goes backwards
			std::cout << "State:5" << endl;
			pos = this->cal_object_position_B_W(scan);
			do_case5(pos, scan);
			
			break;
		case 6:
			std::cout << "State:6" << endl;
			do_case6(scan); break;
			break;
		

		case 7:
			cout << "do case7" << endl;
			pos = this->cal_object_position_B_W(scan);
			std::cout << "(" << sam.landmark_found << "," << pos.position().x() << "," << pos.position().y() << "," << pos.orentation().yaw() << ")" << std::endl;
			if (fabs(pos.orentation().yaw())>15 || pos.orentation().yaw() == 0)
			{

				ofstream savefile(save_file_name, ios::app);
				int size = scan.ranges_size();
				cout << "save_file_name: " << save_file_name << endl;
				cout << "start recording" << endl;
				savefile << "***" << pos.orentation().yaw() << "***""\n";
				for (int i = 0; i < size; i++)
				{
					savefile << scan.ranges(i) << " ";
	
				}

				savefile << "\n";
				savefile.close();
			}
			break;
		case 8:
			//moving backward to leave the dock
			cout << "do case8" << endl;
			do_case8(scan);
		    break; 
		
		case 14:  // 工博会，走直线，x方向速度受控
			pos = this->cal_object_position_B_W(scan);
			//std::cout << "(" << sam.landmark_found << "," << pos.position().x() << "," << pos.position().y() << "," << pos.orentation().yaw() << ")" << std::endl;
			do_case14(pos, scan);
			break;
		
		}
		return;

    }

	void update_goal_reached(const stdmsg::Global_Plan &plan)
	{
		static int goal_reach_count = 0;
		static bool init_switch = true;
		int goal_reached_temp = plan.goal_reached();
		//std::cout << "[BatteryCharge::update_g_r] receive gr from nav: " << _goal_reached << std::endl;
		if (_is_goal_sent && !goal_reached_temp) //grt = 0 igs =1
		{
			std::cout << "[BatteryCharge::update_g_r] waiting for goal reach" << std::endl;
			_goal_reached = 0;
			return;
		}
		else if (_is_goal_sent && goal_reached_temp) // grt=1 igs = 1
		{
			goal_reach_count++;
			if (goal_reach_count >= 3) // real goal reach
			{
				goal_reach_count = 0;
				std::cout << "[BatteryCharge::update_g_r] real goal reach" << std::endl;
				// do what is supposed to do next
				_goal_reached = 1;
				//_is_goal_sent = 0;
				return;
			}

		}
		else
			_goal_reached = 0;
		
		return;

	}

	void update_go_mark(const stdmsg::String &str)
	{
		std::cout << str.str() << std::endl;
		if (str.str() == "go_mark")
		{		
			
			cout << "receive go mark, set state to 2" << endl;
			cout << "===================================" << endl;
			cout << "===================================" << endl; 
			cout << "===================================" << endl;
 			_state = 2;
		}


	}

	//go to the pre-set position in front of the landmark, using SLAM
	void do_case1(const stdmsg::Laser_Scan & scan)
	{
		static int i = 0;
		stdmsg::Pose destination;
		i++;
		if (RECEIVE_CHARGING_REQUEST == 1)
		{
			cout << "[RCR == 1, destination=1]"<<"GR="<<_goal_reached<<"IGS="<<_is_goal_sent;
			destination = goal_pose1;
		}
		else if (RECEIVE_CHARGING_REQUEST == 0)
		{
			//cout << "[RCR == 0, destination=2]";
			//destination = goal_pose2;
			RECEIVE_CHARGING_REQUEST = -1;
			i = 0;
			return;
		}
		else
		{
			
			std::cout << i <<"[waiting for cmd]"<< _goal_reached <<endl;
			i = 0;
			return;
		}

		if (i == 1 && mark2 == 1)
		{
			cout << "set_move_mode" << endl;
			set_move_mode(1);
			cout << "set_goal" << endl;
			set_goal(destination);
			set_goal(destination);
			_sleep(50);
			return;
		}
		
		//if (_goal_reached)
		//{
			//cout << "set_move_mode" << endl;
			//set_move_mode(1);
			//cout << "set_goal" << endl;
			//set_goal(destination);
			//set_goal(destination);
			//_sleep(50);
		if (_goal_reached)
		{

			if (mark2)
			{
				destination = goal_pose3;
				cout << "set_move_mode" << endl;
				set_move_mode(1);
				cout << "set_goal" << endl;
				set_goal(destination);
				set_goal(destination);
				_sleep(50);
				//_sleep(2000);
				mark2 = 0;
			}
			else
			{	
				//int count2 = 0;
				count2++;
				if (count2 > 30)
				{

				if (TERMINATE_CHARGE == 1)
				{
					TERMINATE_CHARGE = 0;
					RECEIVE_CHARGING_REQUEST = -1;
					_is_goal_sent = 0;
					i = 0;
					stdmsg::Data data;
					data.set_seq(2);
					cout << "publish_finish_flag" << endl;
					this->nh->publish("finish_charging_flag", data);
					//this->nh->publish("finish_charging_flag", data);
					//_goal_reached = 0;
					//i = 0;
				}

				else if (RECEIVE_CHARGING_REQUEST == 1)
				{
					i = 0;
					_state = 2;
					_state = 2;
					_is_goal_sent = 0;
					RECEIVE_CHARGING_REQUEST = -1;
					cout << "goal_reached, _state should be 2" << endl;
					//_goal_reached = 0;
					mark2 = 1;
					count2 = 0;
				}
				}
		}
		}
		cout << endl;
		return;
		
	}
	
	void do_case1_demo_0108(const stdmsg::Laser_Scan &scan)
	{ 
		static int i = 0;
		stdmsg::Pose destination;
		i++;
		if (RECEIVE_CHARGING_REQUEST == 1 && i == 0)
		{
			cout << "[RCR == 1, destination=1]" << "GR=" << _goal_reached << "IGS=" << _is_goal_sent;
			destination = goal_pose1;
		}
		//else if (RECEIVE_CHARGING_REQUEST == 0)
		//{
		//	cout << "[RCR == 0, destination=2]";
		//	destination = goal_pose2;
		//}
		else if (RECEIVE_CHARGING_REQUEST == 0)
		{

			std::cout << i << "[waiting for cmd]" << _goal_reached << endl;
			i = -1;
			return;
		}

		if (i == 1)
		{
			set_move_mode(1);
			set_goal(destination);
			set_goal(destination);
			return;
		}
		if (_goal_reached)
		{

			if (RECEIVE_CHARGING_REQUEST == 1)
			{
				_state = 2;
				//_goal_reached = 0;
				i = 0;
			}

		}
		cout << endl;
		return;
	}
	
	//alignment with the middle line
	void do_case2(const stdmsg::Pose pos, const stdmsg::Laser_Scan & scan)
	{
		// pos is actually pos_wb
		double x_wb = pos.position().x();
		double y_wb = pos.position().y();
		double theta_wb = pos.orentation().yaw();
		double laser_pose = 0.18;
		double final_y = laser_pose* sin(theta_wb / 180.0 * M_PI) + _final_y_compensate;

		int temp = 0;
		//cout << x_wb << '\t' << y_wb << '\t' << theta_wb << endl;
		static int k = 0;
		static int i = 0;
		static int step = 1;
		static int count = 0;
		i++;
		//std::cout << "i= " << i << endl;
		if (i <= 2)
		{
			start_time = clock();
			cout << "move_mode_sent" << endl;
			set_move_mode(4);
			return;
		}

		switch (step){
		case 1:
			if (fabs(-1 * theta_wb + 15) > 2)
			{
				cout << "rotating1" << endl;
				rotate(x_wb, y_wb, -1 * theta_wb, -15); //make_it slow
			}
			else
			{
				temp = k;
				k = i;
				if ((k - temp) == 1)
				{
					count++;
					cout << "count = " << count << endl;
				}
					
				else
					count = 0;

				if (count > 3)
				{
					//set_move_mode(0);
					stdmsg::Velocity cmd;
					cmd.set_v(0);
					cmd.set_w(0);
					this->nh->publish("set_cmd_batterycharge", cmd);
					std::cout << "rorate finish" << std::endl;
					step = 2;
					i = 0;
					k = 0;
				}
			}

			break;
		case 2:
			if (fabs(y_wb - (final_y)) >0.03) 
			{
				// 33cm路标，+0.1
				adjust_y(x_wb, y_wb, theta_wb, final_y); // make it slow
				cout << "adjust_y :" << y_wb << " " << final_y << " " << fabs(y_wb - (final_y)) << endl;

			}
			else
			{ 
				// 如果两次执行该函数 fabs（theta）> 1 都成立， 则count++
				//cout << "no adjust y" << endl;
				temp = k;
				k = i;
				if ((k - temp) == 1)
				{
					count++;
					cout << "count = " << count << endl;
				}
					
				else
					count = 0;


				if (count > 8)
				{
					//set_move_mode(0);
					stdmsg::Velocity cmd;
					cmd.set_v(0);
					cmd.set_w(0);
					this->nh->publish("set_cmd_batterycharge", cmd);
					std::cout << "_state2 finish" << std::endl;
					_state = 3;
					step = 1;
					i = 0;
					k = 0;
				}
			}
			//std::cout << "temp = " << temp << " k= " << k << "count= " << count << endl;
			break;
		}


		return;
	}
	
	// rotate to face the landmark
	void do_case3(const stdmsg::Pose pos, const stdmsg::Laser_Scan & scan)
	{
		double x = pos.position().x();
		double y = pos.position().y();
		double theta = pos.orentation().yaw();
		int temp;

		static int k = 0;
		static int i = 0;
		static int count = 0;
		i++;
		//std::cout << "i= "<<i << endl;
		if (i <= 2)
		{
			set_move_mode(4);
			return;
		}
		
		if (fabs(theta) > 2.5)
		{
			rotate2(x, y, theta, 0.0);
			cout << "state3:rotaing " << endl;
		}
			
		else 
		{	
			// 如果两次执行该函数 fabs（theta）> 1 都成立， 则count++
		    temp = k;
		    k = i;
			if ((k - temp) == 1)
				count++;
			else
				count = 0;
			
		
			if (count > 3)
			{
				//set_move_mode(0);
				stdmsg::Velocity cmd;
				cmd.set_v(0);
				cmd.set_w(0);
				this->nh->publish("set_cmd_batterycharge", cmd);
				std::cout << "_state3 finish" << std::endl;
				//_state = 14;
				_state = 4;
				i = 0;
				k = 0;
			}
			
		}	
		//std::cout << "temp = " << temp << " k= " << k << "count= "<< count<< endl;
		return;

	}

	//Go straight to docking station
	void do_case4(const stdmsg::Pose pos, const stdmsg::Laser_Scan & scan)
	{
		double x = pos.position().x();
		double y = pos.position().y();
		double theta = pos.orentation().yaw();

		static int i = 0;
		i++;
		if (i <= 2)
		{
			set_move_mode(4);
			return;
		}

		//cout << "docking_state = " << docking_state_int << endl;
		// determin docking finished or not via STM32
		get_docking_state();
		cout << "docking_state = " << docking_state_int << endl;
		
		if (!docking_state_int)
			go_straight_to_dock(x, y, theta);
		else
		{
			set_move_mode(0);
			_state = 6;
			i = 0;
		}
			
        
	}

	//2016 Move to Omni, gongbohui
	void do_case14(const stdmsg::Pose pos, const stdmsg::Laser_Scan & scan)
	{
		double x = pos.position().x();
		double y = pos.position().y();
		double theta = pos.orentation().yaw();
		double safe_dis = 0.255;//0.25;

		static int k = 0;
		static int i = 0;
		static int count = 0;
		i++;

		if (i == 1)
		{
			set_move_mode(4);
			return;
		}
		std::cout << "i= " << i << endl;
		int temp;

	
		if (fabs(x - _safe_dis) > 0.003)
		{
			go_straight_to_omni(x, y, theta, _safe_dis);
			cout << "going to omni :" << (x - _safe_dis) <<",theta="<< theta<< endl;
				
		}
			

		else
		{
			// 如果两次执行该函数 fabs（theta）> 1 都成立， 则count++
			temp = k;
			k = i;
			if ((k - temp) == 1)
				count++;
			else
				count = 0;

			if (count > 5)
			{
				set_move_mode(0);
				std::cout << "_state14 finish" << std::endl;
				stdmsg::String str;
				str.set_str("mark_reach");
				this->nh->publish("mark_reach", str);
				_state = 0;
				i = 0;
				k = 0;
			}

		}
		//std::cout << "temp = " << temp << " k= " << k << "count= "<< count<< endl;
		return;

	
	}
	void do_case5(const stdmsg::Pose pos, const stdmsg::Laser_Scan & scan)
	{
		static int i = 0;
		stdmsg::Velocity cmd;
		double v = 0.1;
		double w = 0;
		i++;
		if (i == 1)
		{
			set_move_mode(4);
			return;
		}

		else if (i < 20)
		{
			cmd.set_v(v);
			cmd.set_w(w);
			this->nh->publish("set_cmd_batterycharge", cmd);
			return;
		}
		
		_state = 6;
	
	
	}
	void do_case6(const stdmsg::Laser_Scan & scan)
	{
		static int i = 0;
		static int k = 1;
		i++;
		cout << "docking process finished" << endl;

		if (i <= 1)
		{
			ofstream logfile(str_logfile, ios::app);
			RECEIVE_CHARGING_REQUEST = 0;
			set_move_mode(0);
			clock_t end_time = clock();
			logfile <<k<<" " <<static_cast<double>(end_time - start_time) / CLOCKS_PER_SEC<<endl;
			k++;
			logfile.close();
		}
		//if (docking_state_int == 0)
		//{
		//	_state = 4;
		//}
		else if (TERMINATE_CHARGE == 1)
		{
			_state = 8;
			i = 0;
		}
			
		return;
	}
	void do_case8(const stdmsg::Laser_Scan & scan)
	{
		static int i = 0;
		i++;
		cout << i << endl;
		if (i <= 1)
		{
			set_move_mode(4);
		}
		
		else if (i <= 60)
		{
			stdmsg::Velocity back_cmd;
			double v = -0.06;
			double w = 0;
			back_cmd.set_v(v);
			back_cmd.set_w(w);
			this->nh->publish("set_cmd_batterycharge", back_cmd);
		}
		else
		{
			set_move_mode(0);			
			_state = 1;
			RECEIVE_CHARGING_REQUEST = 0;
			i = 0;
		}
	
		return;
	}
	void rotate(double x, double y,double theta, double final_angle)
	{
		stdmsg::Velocity cmd;
		static int zero_count = 0;
		double v = 0.0, w = 0.0;
		double error_theta;
		if (fabs(theta) > 50)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		else if (x == 0 && y == 0 && theta == 0)
		{
			zero_count++;
			if (zero_count > 10)
			{
				std::cout << "set velocity = 0" << std::endl;
				v = 0;
				w = 0;
			}
			
		}

		else
		{
			zero_count = 0;
			error_theta = theta- final_angle;
			v = 0.0;
			//w = error_theta*M_PI / 180 * 1;
			if (error_theta > 0)
				//w = 0.1;
				w = 0.2;
			else
				//w = -0.1;
				w = -0.2;
			std::cout << "tehta,w = " << theta << ", " << w << endl;
			if (w > nav_w())
				w = nav_w();
			if (w < -nav_w())
				w = -nav_w();
			std::cout<< "w has been set"<<endl;
		}

		cmd.set_v(v);
		cmd.set_w(w);
		//std::cout << "err_d,err_a = " << error_distance << "," << error_theta << std::endl;
		//std::cout<<"velocity"<<" v="<<v<<" w="<<w<<std::endl;
		this->nh->publish("set_cmd_batterycharge", cmd);
		return;
	}
	void rotate2(double x, double y, double theta, double final_angle)
	{
		stdmsg::Velocity cmd;
		static int zero_count = 0;
		double v = 0.0, w = 0.0;
		double error_theta;
		if (fabs(theta) > 50)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		else if (x == 0 && y == 0 && theta == 0)
		{
			zero_count++;
			if (zero_count > 10)
			{
				std::cout << "set velocity = 0" << std::endl;
				v = 0;
				w = 0;
			}

		}

		else
		{
			zero_count = 0;
			error_theta = theta - final_angle;
			v = 0.0;
			//w = error_theta*M_PI / 180 * 1;
			if (error_theta > 0)
				//w = 0.1;
				w = 0.2;
			else
				//w = -0.1;
				w = -0.2;
			std::cout << "tehta,w = " << theta << ", " << w << endl;
			std::cout << "w2 has been set" << endl;
			if (w > nav_w())
				w = nav_w();
			if (w < -nav_w())
				w = -nav_w();
		}

		cmd.set_v(v);
		cmd.set_w(w);
		//std::cout << "err_d,err_a = " << error_distance << "," << error_theta << std::endl;
		//std::cout<<"velocity"<<" v="<<v<<" w="<<w<<std::endl;
		this->nh->publish("set_cmd_batterycharge", cmd);
		return;
	}
	void go_straight_to_dock(double x, double y, double theta)
	{
		stdmsg::Velocity cmd;
		double v = 0.0, w = 0.0;
		double stop_delta_distance = 0.23;
		double error_theta, error_distance;
		double p1, p2;
		int sign = 0;

		if (fabs(theta) > 50)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		else if (x == 0 && y == 0 && theta == 0)
		{
			std::cout << "no landmark ,set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		//if(pos.orentation().yaw()<0)

		else
		{
			error_theta = atan(y / x);
			//error_distance = sqrt(x*x + y*y) - safe_dis + 0.1;
			//error_distance = sqrt(x*x + y*y) ;
			error_distance = x - _safe_dis;
			while (error_theta> 3.141592653) error_theta -= 3.141592653 * 2;
			while (error_theta<-3.141592653) error_theta += 3.141592653 * 2;

			if (error_distance > 0.15 && error_distance < 5)
			{
				//v = nav_v()*error_distance / 3;
				v = 0.06;
			}
			else if (error_distance > 0 && error_distance < 0.15)
			{
				//v = nav_v()*error_distance ;
				v = 0.06;
			}
			else if (error_distance < 0){
				v = 0.0;
			}

			//p1 = 0.3;
			//p2 = 0.4 ;
			w = _py*y + _pw*theta*M_PI / 180;
			//
			//w = error_theta * 0.5;
			//if (w > nav_w())
			//	w = nav_w();
			//if (w < -nav_w())
			//	w = -nav_w();

			// if (fabs(error_theta) > stop_delta_theta()*3.141592653 / 180)

		}

		if (v > 0.1){ v = 0.1; }
		if (fabs(w) > 0.5)
		{
			sign = (w > 0) ? 1 : -1;
			w = 0.5*sign;
		}

		cmd.set_v(v);
		cmd.set_w(w);
		//std::cout << "err_d,err_a = " << error_distance << "," << error_theta << std::endl;
		std::cout<<"velocity"<<" v="<<v<<" w="<<w<<std::endl;
		this->nh->publish("set_cmd_batterycharge", cmd);
		return;
	}
	void go_straight_to_omni(double x, double y, double theta, double safe_dis)
	{
		stdmsg::Velocity cmd;
		double v = 0.0, w = 0.0;
		double stop_delta_distance = 0.23;
		double error_theta, error_distance;
		
		double p1, p2;

		int sign = 0;

		if (fabs(theta) > 10)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		else if (x == 0 && y == 0 && theta == 0)
		{
			std::cout << "no landmark ,set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		//if(pos.orentation().yaw()<0)

		else
		{
			error_theta = atan(y / x);
			//error_distance = sqrt(x*x + y*y) - safe_dis;
			error_distance = x - safe_dis;
			//error_distance = sqrt(x*x + y*y);

			while (error_theta > 3.141592653) error_theta -= 3.141592653 * 2;
			while (error_theta < -3.141592653) error_theta += 3.141592653 * 2;

			if (error_distance > 0.2 && error_distance < 5)
			{
				v = nav_v()*error_distance*0.6;
				//v = 0.08;
			}
			else if (error_distance < 0.2)
			{
				
				v = nav_v()*error_distance * 0.3;
				//v = 0.05;
			}
			

			p1 = 0.8;  //0.3
			p2 = 0.3;
			w = p1*y + p2*theta*M_PI / 180;


			if (v > 0.1){ v = 0.1; }
			if (fabs(w) > 0.5)
			{
				sign = (w > 0) ? 1 : -1;
				w = 0.5*sign;
			}

			//w = 0; // only set v
			cmd.set_v(v);
			cmd.set_w(w);
			//std::cout << "err_d,err_a = " << error_distance << "," << error_theta << std::endl;
			std::cout<<"[go_straiht_to_omni]= "<<"("<<v<<","<<w<<")"<<std::endl;
			this->nh->publish("set_cmd_batterycharge", cmd);
			return;

		}
	}

	void get_docking_state()
	{
		stdmsg::SomeFlag tmp1, tmp2;
		tmp1 = rpc.call<stdmsg::SomeFlag, stdmsg::SomeFlag>("get_docking_state", tmp2);
		//docking_state.set_emergency_stop_flag(tmp1.emergency_stop_flag());
		docking_state_int = tmp1.emergency_stop_flag();
	}

	void adjust_y(double x_wb, double y_wb, double theta_wb,double final_y)
	{
		stdmsg::Velocity cmd;
		
		double v = 0.0, w = 0.0;
		double error_y = y_wb - final_y;
		static int zero_count = 0;
		if (fabs(theta_wb) > 50)
		{
			std::cout << "set velocity = 0" << std::endl;
			v = 0;
			w = 0;
		}
		else if (x_wb == 0 && y_wb == 0)
		{
			zero_count++;
			std::cout << "no landmark, set velocity = 0" << std::endl;
			if (zero_count > 10)
			{
				v = 0;
				w = 0;
			}
			
		}

		else
		{
			zero_count = 0;
			w = 0.0;
			int kp = 0.8;
			if (error_y > 0)
				//v = -1*(error_y*error_y) * kp;
				v = -0.03;
			else
				//v = (error_y*error_y) * kp;
				v = 0.03;

			//std::cout << "v = " << v << endl;

			cmd.set_v(v);
			cmd.set_w(w);
			//std::cout << "err_d,err_a = " << error_distance << "," << error_theta << std::endl;
			//std::cout<<"final_y"<< final_y<<" velocity"<<" v="<<v<<" w="<<w<<std::endl;
			this->nh->publish("set_cmd_batterycharge", cmd);
			return;
		}

	}

	~BatteryCharge()
    {
        std::cerr<<"node uninitialize!"<<std::endl;
        if(nh) delete nh;
    }
};

int main(int argc, char **argv)
{
    BatteryCharge F(argc, argv);
	while(1){_sleep(360000);}
}
