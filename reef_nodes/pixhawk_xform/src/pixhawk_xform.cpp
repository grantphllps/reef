//
// pixhawk_xform.cpp
//
// This node flips the axes on the mocap system
// so the correct positional data can be streamed
// to the Pixhawk via mavros
//
// Created by Adam Plowcha, modified by Grant Phillips
//

#include <pixhawk_xform.h>

int zMod = 1;

int main(int argc, char** argv)
{

    //Default topic name...
    std::string topic_name = "/vicon/CROOKSHANKS/CROOKSHANKS";

    if (argc >1)
    {
        topic_name = "/vicon/";
        topic_name = topic_name + argv[1] +"/" + argv[1];
        ROS_INFO("Pixhawk_xform using topic %s.", topic_name.c_str());
    }

    zMod = atoi(argv[4]);

    //initialize ros
    ros::init(argc, argv, "pixhawk_xform");
    ros::NodeHandle nh;

    ROS_INFO("Pixhawk_xform starting...");

    //Setup pubs/subs/clients
 
    //Main output poseStamped required by mavros
    xform_pub = nh.advertise<geometry_msgs::PoseStamped>("pixhawk_xform/pose_stamped", 20);
    
    //Vicon subscription
    vicon_sub = nh.subscribe(topic_name, 10, callbackVicon);
    
    //main processing loop
    while (nh.ok())
    {

        ros::spinOnce();

    }

}

void callbackVicon(const geometry_msgs::TransformStampedConstPtr &msg)
{
	//Vicon outputs a TransformStamped message and the fake_gps
	//mavros message requires a PoseStamped message.  This method
	//simply takes the relevant info from the TransformStamped
	//and translates it to a PoseStamped which is then published
	//to pixhawk_xform/pose_stamped at 20 hertz
	ros::Rate rate(30);
	
    vicon = *msg;
    	
    xformed.header = vicon.header;
    	
    xformed.pose.position.x = vicon.transform.translation.x;
   	xformed.pose.position.y = vicon.transform.translation.y;
   	xformed.pose.position.z = zMod*(vicon.transform.translation.z);
   	
   	xformed.pose.orientation = vicon.transform.rotation;
   	
   	xform_pub.publish(xformed);
   	rate.sleep();
}


//This is a helpful node...
geometry_msgs::Pose getDegFromQuat(geometry_msgs::Pose p)
{
    geometry_msgs::Pose retP;

    //Change this later after we figure out the NED/world frame stuff
    retP.position.x = p.position.x;
    retP.position.y = p.position.y;
    retP.position.z = p.position.z;

    tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    retP.orientation.x = rad_to_deg(pitch);
    retP.orientation.y = rad_to_deg(roll);
    retP.orientation.z = rad_to_deg(yaw);

    return retP;
}


