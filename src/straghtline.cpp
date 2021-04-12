
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>


void robot_odom(const nav_msgs::Odometry msg);                      // get robot position
double getDistance(double x1, double y1, double x2, double y2);    //position error


ros::Publisher robot_vel_pub,errors_pub,desired_traj_pub;           // node publishers
tf::Point Odom_pos;                                                //odometry position (x,y)
double Odom_yaw;                                                   //odometry orientation (yaw)
geometry_msgs::Pose2D qd,robot_pose,err;
geometry_msgs::Twist vel_msg;

double	kvp = 2.25;
double	kwp = 1.5;
double	kvd = 0.0;
double  kwd = 0.0;
double  kvi = 0.0;
double  kwi = 0.0;
double xd,yd;
double	ev,ew,ev_prev,ew_prev,pl,dl,pa,da,il,ia = 0.0;
double  vr,wr,t;
double  freq=2*M_PI/35;
double  vm = 0.22; // Turtlebot3 maximum linear velocity
double  wm = 2.8;  // Turtlebot3 maximum angular velocity

//parameter: ideal distance for moving 
double distance=1;



int main(int argc, char **argv)
{

    ROS_INFO("start");
    ros::init(argc, argv, "PID_trajectory_tracking_node");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/odom", 1000 , robot_odom);
    robot_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	errors_pub = n.advertise<geometry_msgs::Pose2D>("/errors_pub",1000);
	desired_traj_pub = n.advertise<geometry_msgs::Pose2D>("/desired_traj_pub",1000);
	ros::Rate loop_rate(10); 

	// wait for all publishers and subscribers to connected to avoid losing msgs
	while (errors_pub.getNumSubscribers() == 0 || desired_traj_pub.getNumSubscribers() == 0 
		|| robot_vel_pub.getNumSubscribers() == 0 )
	{
		loop_rate.sleep();
	}

    vr=0.1;                           //initial speed                         
    double start_x=robot_pose.x;      //robot start point
    double start_y=robot_pose.y;

    qd.x=-0.2;                      
    qd.y=0.2;
    qd.theta=0.785;

    double del_t=0.1;

	while(getDistance(start_x,start_y,robot_pose.x,robot_pose.y)<1 && ros::ok()){

		qd.x=qd.x+vr*del_t*cos(qd.theta);
        qd.y=qd.y+vr*del_t*sin(qd.theta);

		desired_traj_pub.publish(qd); 

		err.x = (qd.x-robot_pose.x);
        err.y=(qd.y-robot_pose.y);
		errors_pub.publish(err); 

		ev_prev = ev;

		pl = kvp * ev;
		dl = kvd * ((ev - ev_prev)/0.1);
		il = kvi * ((ev + ev_prev)*0.1);

		ev = getDistance(robot_pose.x, robot_pose.y, qd.x, qd.y);
		vr = pl + il + dl;
		if (vr > vm )
		{	
			vr = vm;     
		}

		if (wr > wm )
		{	
			wr = wm;
		}			
				
		vel_msg.linear.x = vr;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =0;
		robot_vel_pub.publish(vel_msg);            // publish the robot velocities
		ros::spinOnce();
		loop_rate.sleep();
	};

	//stop robot
	vel_msg.linear.x = 0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;
	robot_vel_pub.publish(vel_msg);
    return 0;
}



double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

// function to get robot position and orientation from the odom topic
void robot_odom(const nav_msgs::Odometry msg)
{
    tf::pointMsgToTF(msg.pose.pose.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.pose.pose.orientation);
	robot_pose.x = msg.pose.pose.position.x;
	robot_pose.y = msg.pose.pose.position.y;
    robot_pose.theta = Odom_yaw;   
}
