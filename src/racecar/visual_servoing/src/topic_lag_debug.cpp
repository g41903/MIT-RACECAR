
/*
This C++ node tracks the propagation time and packet drop rate of a sequence of Image messages.

Winter Guerra
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <stdio.h>

// Here are some of our custom datatypes.
// #include "visual_servoing/CameraObject.h"
#include "visual_servoing/CameraObjectsStamped.h"

// Our variables
std::string upstream_topic, downstream_topic, output_topic;
std::string node_name = "debug_node";
ros::Publisher debug_output;

ros::Time timestamp_of_last_received_upstream_packet;
ros::Time ros_time_of_receipt;
bool waiting_for_upstream_packet = true; 


void upstreamCallback(const std_msgs::Header& message){
	// Let's log the header of this message
	ros::Time original_packet_timestamp = message.stamp;


	if (waiting_for_upstream_packet || original_packet_timestamp < timestamp_of_last_received_upstream_packet){
		timestamp_of_last_received_upstream_packet = original_packet_timestamp;
		ros_time_of_receipt = ros::Time::now();
		waiting_for_upstream_packet = false;
	}

	//ROS_INFO("I heard the upstream header [%i]", int(seq_num));
}

void downstreamCallback(const std_msgs::Header& message){
	// Let's log the header of this message
	ros::Time original_packet_timestamp = message.stamp;

	// calculate the propagation delay.
	if (original_packet_timestamp == timestamp_of_last_received_upstream_packet){
		ros::Duration t_propagation = ros::Time::now() - ros_time_of_receipt;
		ros::Duration t_upstream_delay = ros_time_of_receipt - timestamp_of_last_received_upstream_packet;
		
		// Log our result.
		std_msgs::String report;
		std::stringstream temp_stream;
		temp_stream << "The propagation delay in ms is: " << t_propagation.toSec()*1000 << "\n The delay from the creation of the initial packet waay upstream to the upstream topic in ms is: " << t_upstream_delay.toSec()*1000;
		report.data = temp_stream.str();
		debug_output.publish(report);

	
	// We have dropped a packet!
	} else if (original_packet_timestamp > timestamp_of_last_received_upstream_packet){
		//log the dropped packet
		waiting_for_upstream_packet = true;
	
	// If we have been waiting for a long time, then tell the user that something may be wrong.
	} else if ((ros::Time::now() - ros_time_of_receipt).toSec() > 1.0) {
		std_msgs::String report;
		std::stringstream temp_stream;
		temp_stream << "WARNING! Packets seem to be taking more than 1 second to propagate. \n This seems very unlikely. Therefore, the obvious conclusion is that your upstream and downstream topics are not using the same header timestamps. \n Have you tried using rostopic on the upstream and downstream topics to see that the timestamps match? NOTE: the sequence numbers of the packets will most definitely not match since ROS overwrites this.";
		temp_stream << "\nThe downstream packet timestamp is " << original_packet_timestamp.toSec() << " The upstream packet timestamp is " << timestamp_of_last_received_upstream_packet.toSec();
		report.data = temp_stream.str();
		debug_output.publish(report);

	}

	//ROS_INFO("I heard the downstream header [%i]", int(seq_num));

}


int main(int argc, char **argv)
{
	// Init ROS
	// ros::init_options::AnonymousName will add a unique identifier to the end of your node name.  This allows
	// multiple of the same executable to be run without remapping each of their names with
	// __name:=X
	ros::init(argc, argv, node_name); //Note: Node_name can be overridden by the launch file
	ros::NodeHandle nh;
	ros::NodeHandle node_param("~");


	// Figure out which topics we should subscribe to
	if ( !(node_param.getParam("upstream_topic", upstream_topic) && node_param.getParam("downstream_topic", downstream_topic) && node_param.getParam("output_topic", output_topic)) ) {
		throw std::runtime_error("Error! upstream_topic, downstream_topic, and output_topic params are required in the launch file for running topic_lag_debug node types");
	}

	// Let's publish our results
	debug_output = nh.advertise<std_msgs::String>(output_topic, 1);

	// Let's subscribe to our 2 topics so that we can measure the propagation delay.
	ros::Subscriber sub_upstream = nh.subscribe(upstream_topic, 1, upstreamCallback);
	ros::Subscriber sub_downstream = nh.subscribe(downstream_topic, 1, downstreamCallback);

	ros::spin();

		return 0;
}