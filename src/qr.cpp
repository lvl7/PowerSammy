#include <ros/ros.h>
//#include <string>
//#include <sensor_msgs/LaserScan.h>
//#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

void qrMessReceived(std_msgs::String);
void posMessReceived(geometry_msgs::PoseStamped msg);

ros::NodeHandle *node;
ros::Publisher velocityPub;
ros::Publisher cameraPub;
ros::Subscriber poseSub;
ros::Subscriber qrMessSub;

int status = 0;
// 0: czekaj na kod qr, jesli znajdzie i wczyta dobre dane, przejdz dalej
// 1: zapisz poczatkowe ustawienie i kat, oblicz docelowy kat, przejdz dalej
// 2: obrot az aktualny kat = docelowy kat obrotu, przejdz dalej
// 3: rucha az przejechana odleglosc = zadana, dalej
// 4: obrot jak w pkcie 2. o drugi kat, KONIEC (status = 0) 

double distToMove = 0;      // o ile ma sie ruszyc (z kodu qr, metry)
double initX = 0;           // poczatkowa wspolrzedna x przy ruchu
double initY = 0;           // pocatkowa wspolrzedna y przy ruchu

double angToMove = 0;       // o jaki kat ma sie obrocic na poczatku (z kodu qr, stopnie)
double angToMoveFinal = 0;  // o jaki kat am sie obrocic na koncu (z kodu qr, stopnie)
double targetAng = 0;     // docelowy kat przy obracaniu sie (stopnie)
double initAng = 0;         // poczatkowy kat przy obracaniu sie (stopnie)

ros::Timer timeoutTimer;

// FUNKCJA ZATRZYMUJACA RUCH I RESETUJACA STATUS DO WSADZENIA POD TIMER
void timeoutStop(const ros::TimerEvent& e)
{
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = vel.linear.z = 0;
    vel.angular.x = vel.angular.y = vel.angular.z = 0;
	velocityPub.publish(vel);
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("!!! TIMEOUT !!!");
	ROS_INFO_STREAM("");
	status = 0;
}

// FUNKCJA ZATRZYMUJACA RUCH DO WYWOLANIA
void stopMove()
{
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = vel.linear.z = 0;
    vel.angular.x = vel.angular.y = vel.angular.z = 0;
	velocityPub.publish(vel);
}



// modul liczby
double mabs(double arg){ return (arg>0)?arg:arg*-1; }

void poseDataReceived(nav_msgs::Odometry msg)
{
	// przelicz aktualny kat i pozycje
	tf::Quaternion quat(msg.pose.pose.orientation.y, msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Matrix3x3 mat(quat);
	double r, p, y;
	mat.getRPY(r, p, y);
	// aktualne wspolrzedne
	double currAng = y*360/(2*3.14);
	double currX = msg.pose.pose.position.x;
	double currY = msg.pose.pose.position.y;
	
	if(status == 1) 
	{
		// zapisz poczatkowe wspolrzedne
		initX = msg.pose.pose.position.x;
		initY = msg.pose.pose.position.y;
		initAng = currAng;

		// oblicz docelowy kat, sprowadz do zakresu (-180 : 180)
		targetAng  = initAng + angToMove;
	    while(mabs(targetAng) > 180)
			targetAng += 360*((targetAng>0)?-1:1);

		// nastaw timer, 5s + 10s na każde 90 stopni obrotu
		double maxRotationTime = 5 + mabs(angToMove)*10/90;
		timeoutTimer.stop();
		timeoutTimer.setPeriod(ros::Duration(maxRotationTime));
		timeoutTimer.start();
		status = 2;
	}
	else if(status == 2) // obrot do zadanego kata
	{
		ROS_INFO_STREAM("CURR:");
		ROS_INFO_STREAM(currAng);
		ROS_INFO_STREAM("TARGET:");
		ROS_INFO_STREAM(targetAng);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(mabs(targetAng - currAng));
		geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		
		if(mabs(targetAng - currAng) > 1) // dopoki roznica > 1 stopnien
		{
			vel.angular.z = 0.5;
			if(angToMove < 0) vel.angular.z = -1*vel.angular.z;
			if(mabs(targetAng - currAng) < 30)
				vel.angular.z = vel.angular.z*(mabs(targetAng - currAng)/30);
			//ROS_INFO_STREAM("VEL:");
			//ROS_INFO_STREAM(vel.angular.z);
			velocityPub.publish(vel);
		}
		else
		{
			stopMove();

			// resetuj i nastaw timer, 5s + 5s na kazdy metr do przejechania
			double maxMoveTime = 5 + 5*distToMove;
			timeoutTimer.stop();
			timeoutTimer.setPeriod(ros::Duration(maxMoveTime));
			timeoutTimer.start();
			
			status = 3;
		}
		
	}
	else if(status == 3)
	{
		// oblicz ile aktualnie przejechane
		double distX = initX - currX;
		double distY = initY - currY;
		double currDist = sqrt(pow(distX,2) + pow(distY,2));
		
		ROS_INFO_STREAM("CURR DIST:");
		ROS_INFO_STREAM(currDist);
		ROS_INFO_STREAM("TARGET DIST:");
		ROS_INFO_STREAM(distToMove);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(distToMove - currDist);

		geometry_msgs::Twist vel;
	    vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;

		if( ( mabs(distToMove - currDist) > 0.05 ) && ( distToMove != 0 ) )
		{
			vel.linear.x = 0.3;
			if(mabs(distToMove - currDist) < 0.4)
				vel.linear.x = vel.linear.x*((distToMove - currDist)/0.4); // bylo 0.4
			if((distToMove - currDist) < 0)
				vel.linear.x = vel.linear.x * -1;
			velocityPub.publish(vel);
			
		}
		else
		{
			timeoutTimer.stop();
			//timeoutTimer.setPeriod(ros::Duration(0.5));
			//timeoutTimer.start();
			
			if(angToMoveFinal == 0) // jesli nie bylo zadanego koncowego obrotu
			{
				stopMove();
				status = 0;
				std_msgs::Float64 msg;
				msg.data = 6;
				cameraPub.publish(msg);
			}
			else
			{
				angToMove = angToMoveFinal;
				angToMoveFinal = 0;
				distToMove = 0;
				status = 1;
				}
			/*stopMove();
			double data = 2.79253 + angToMoveFinal*360/(2*3.14);
			std_msgs::Float64 msg;
			msg.data = data;
			//cameraPub.publish(msg);
			status = 6;*/
	    
		}
		
	}
	
}

void qrDataReceived(std_msgs::String msg)
{
	timeoutTimer.stop();
	
	if(status==0)
	{
		ROS_INFO_STREAM(msg.data);
		std::string qr_msg;
		if(msg.data == std::string("D"))
		{ 
			distToMove = 2.15;
			angToMove = -120;
			angToMoveFinal = 30;	
		}
		else if(msg.data == std::string("B"))
		{ 
			distToMove = 2;
			angToMove = 180;
			angToMoveFinal = 45;
		}
		else if(msg.data == std::string("C"))
		{ 
			distToMove = 1;
			angToMove = -135;
			angToMoveFinal = 0;
		}
		else
		{ 
			distToMove = 0;
			angToMove = 0;
			angToMoveFinal = 0;
   
		}
		if((distToMove != 0)&&(angToMove!=0)) status = 1;	
	}
}

void sigHandler(int sig)
{
	ROS_INFO_STREAM("SIG");
	stopMove();
	ros::shutdown();
}

int main (int argc, char** argv)
{
	signal(SIGINT, &sigHandler);
	ros::init(argc, argv, "qr", ros::init_options::NoSigintHandler);
	node = new ros::NodeHandle;
	ros::Rate rate(30);

	velocityPub = node->advertise<geometry_msgs::Twist>("/rosaria/cmd_vel", 100);
	cameraPub = node->advertise<std_msgs::Float64>("/nie_controller/command", 100);
	poseSub = node->subscribe("/rosaria/pose", 100, &poseDataReceived);
	qrMessSub = node->subscribe("/visp_auto_tracker/code_message", 100, &qrDataReceived);

	timeoutTimer = node->createTimer(ros::Duration(10), timeoutStop, true);
	timeoutTimer.stop();
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	ros::spin();
	return 0;
}
