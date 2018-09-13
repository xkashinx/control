#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>
#include "control/sideWay.h"
#include "control/angleDistanceError.h"
#include "control/pid_input.h"
#define dataSize (1081)
#define validSize (150) 
#define ratio (1)//(7.525)
#define real (true)
typedef struct
{
	double x,y;
} Point;

std::vector<Point> rightWall,leftWall;
ros::Publisher pub,pubError,pub2Python;


Point convertCoord(double range,int index)
{
	double theta = (-135+index*0.25)*3.14159265358979323846264/180.0;
	Point ans;
	ans.x = range*cos(theta);
	ans.y = range*sin(theta);
	return ans;
}
inline double distanceOfPoints(Point A,Point B)
{
	return sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
}

void linearFit(std::vector<Point> P,double &A,double &B)
{
	std::vector<Point>::iterator it;
	double Sxy=0,Sx=0,Sy=0,Sxx=0,N;
	N=P.size();
	A=0;
	B=0;
	for (it=P.begin();it!=P.end();it++)
	{
		Sxy += it->x*it->y;
		Sx  += it->x;
		Sy  += it->y;
		Sxx += it->x*it->x;
	}
	A = (Sxy-Sx*Sy/N)/(Sxx-Sx*Sx/N);
	B = Sy/N-A*Sx/N;
	
//	ROS_INFO("Xave:%lf Yave:%lf A:%lf B:%lf C:%lf",Xave,Yave,A,B,C);
}
void calculateAngleDistance(double A,double B,double &ang, double &dist)
{
	ang = -atan(A);
	dist = cos(ang)*B;
	ang = ang*180/3.14159265;
}

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//	ROS_INFO("%d",sizeof(msg->ranges));
	//ROS_INFO("Message received");
	leftWall.clear();
	rightWall.clear();

	int outrange_count =0;
	Point lastPoint,currentPoint;

	for (int i=1;i<dataSize;i++)
	{	
		if (msg->ranges[i]>4*ratio && msg->ranges[i]<100 && rightWall.size()>0) outrange_count++;

		if (outrange_count>20) break;

		if (msg->ranges[i]<3*ratio)
		{
			currentPoint = convertCoord(msg->ranges[i],i);
			if (rightWall.size()>0 && distanceOfPoints(currentPoint,lastPoint)>0.3*ratio)
				break;
			rightWall.push_back(currentPoint);
			lastPoint = currentPoint;
		}
	}

	outrange_count=0;
	
	for (int i=dataSize-1;i>1;i--)
	{	
		if (msg->ranges[i]>4*ratio && msg->ranges[i]<100 && leftWall.size()>0) outrange_count++;
		
		if (outrange_count>20) break;

		if (msg->ranges[i]<3*ratio)
		{
			currentPoint = convertCoord(msg->ranges[i],i);
			if (leftWall.size()>0 && distanceOfPoints(currentPoint,lastPoint)>0.3*ratio)
				break;
			leftWall.push_back(currentPoint);
			lastPoint = currentPoint;
		}
	}


	ROS_INFO("%ld %ld",leftWall.size(),rightWall.size());

	control::sideWay side;
	control::angleDistanceError error;
	control::pid_input pid_error;


	linearFit(leftWall,side.LA,side.LB);
	linearFit(rightWall,side.RA,side.RB);
	double Lang,Rang,Ldist,Rdist;

	calculateAngleDistance(side.LA,side.LB,Lang,Ldist);
	calculateAngleDistance(side.RA,side.RB,Rang,Rdist);

	if (leftWall.size()>validSize && rightWall.size()>validSize)
	{
		error.ang  =  (Lang*leftWall.size()+Rang*rightWall.size())/(leftWall.size()+rightWall.size());
		error.dist = -(Ldist*leftWall.size()+Rdist*rightWall.size())/(leftWall.size()+rightWall.size());
	}
	
	if (leftWall.size()<=validSize && rightWall.size()>validSize)
	{	
		error.ang  =  Rang;
		error.dist =  -1*ratio+(-Rdist);
	}

	if (leftWall.size()>validSize && rightWall.size()<=validSize)
	{

		error.ang  =  Lang;
		error.dist =  (1*ratio-Ldist);
	}

	if (leftWall.size()<=validSize && rightWall.size()<=validSize)
	{

		error.ang  =  0;//invalid
		error.dist =  0;
	}
	ROS_INFO("La:%0.5lf Ra:%0.5lf Ld:%0.5lf Rd:%0.5lf",Lang,Rang,Ldist,Rdist);
	ROS_INFO("Ea:%0.5lf Ed:%0.5lf",error.ang,error.dist);
	
	
	pid_error.pid_error =(error.dist/ratio+error.ang/45*1.5)*100;
	pid_error.pid_vel = 9*ratio;
	if (ratio >= 1.1 && pid_error.pid_vel > 50.0) pid_error.pid_vel = 50.0; // don't let vehicle overturn 
	if (pid_error.pid_vel<2*ratio) pid_error.pid_vel=2*ratio;

	pub.publish(side);
	pubError.publish(error);
	pub2Python.publish(pid_error);
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"side_way_controller");
	ROS_INFO("Side Way Finder Start");
	ros::NodeHandle rosHandle;
	ros::Subscriber sub = rosHandle.subscribe("/scan", 100, callback);
	//ros::Subscriber sub = rosHandle.subscribe("catvehicle/front_laser_points",100,callback);
	pub = rosHandle.advertise<control::sideWay>("control/sideWay",100);
	pubError = rosHandle.advertise<control::angleDistanceError>("control/angleDistanceError",100);	
	pub2Python = rosHandle.advertise<control::pid_input>("control/error",100);

	ros::spin();
	return 0;
}
