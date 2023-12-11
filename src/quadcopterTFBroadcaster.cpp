#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

ros::Time lastStamp;
using std::cout; using std::endl;
void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;	
	geometry_msgs::TransformStamped transformStamped;
	
	if (lastStamp != ros::Time::now()){
		transformStamped.header.stamp = ros::Time::now();
		lastStamp = transformStamped.header.stamp;
		transformStamped.header.frame_id = msg->header.frame_id;
		transformStamped.child_frame_id = "base_link";
		transformStamped.transform.translation.x = msg->pose.position.x;
		transformStamped.transform.translation.y = msg->pose.position.y;
	    transformStamped.transform.translation.z = msg->pose.position.z;

		transformStamped.transform.rotation.x = msg->pose.orientation.x;
		transformStamped.transform.rotation.y = msg->pose.orientation.y;
		transformStamped.transform.rotation.z = msg->pose.orientation.z;
		transformStamped.transform.rotation.w = msg->pose.orientation.w;
		br.sendTransform(transformStamped);

	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "quadcopter_tf_broadcaster");
	ros::NodeHandle nh;
	std::string poseTopic = "/CERLAB/quadcopter/pose";
	if (argc > 1)  poseTopic = argv[1];
	// cout << "poseTopic: " << poseTopic << endl;
	ros::Subscriber sub = nh.subscribe(poseTopic, 1, &poseCallback);
	ros::spin();
	return 0;
}