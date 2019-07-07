#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

// Global state variable
int STATE;


void callback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  float error_margin = 0.3;
//  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("STATE: %d\n", STATE);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

  switch (STATE)
  {
    case 0: // this is the initialised case. The robot is not yet at the pickup zone.
      // Check to see if we are at the pickup zone
      // pickup is x=2, y=2, z=0
      if (msg->pose.pose.position.x >= 2.0-error_margin && msg->pose.pose.position.x <= 2.0+error_margin && msg->pose.pose.position.y >= 2.0-error_margin && msg->pose.pose.position.y <= 2.0+error_margin)
      {
        ROS_INFO("AT Pickup zone!");
        STATE = 1; // set to be the next state
      }
      break;
    case 1: // This state is just to delete the marker and wait 5 seconds. Do nothing.
      break;
    case 2: // This is the case where we have reached the pickup zone, but not yet reached the drop off zone.      
      // Check to see if we are at the drop off zone
      // drop off is x=-1, y=-1, z=0
      if (msg->pose.pose.position.x >= 0.0-error_margin && msg->pose.pose.position.x <= 0.0+error_margin && msg->pose.pose.position.y >= 0.0-error_margin && msg->pose.pose.position.y <= 0.0+error_margin)
      {
        ROS_INFO("AT Drop off zone!");  
        STATE = 3; // set to be the next state        
      }
      break;
    case 3: // this is the state where the marker was added to the drop off zone. Do nothing.
      break;
    case 4: // this is the 'final' state. 
      break;
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber subOdom = n.subscribe("odom", 1000, callback);
  
  ////////////////////////////////////////////
  // Start with the marker at the pickup zone.
  ////////////////////////////////////////////
  
  
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type. 
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 2.0;
  marker.pose.position.y = 2.0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
  

  while(ros::ok())
  {    
    switch (STATE)
    {
      case 0: // do nothing - still moving to the pickup zone.
        break;
      case 1: // got to the pickup zone remove the marker and wait 5 seconds.
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ros::Duration(5.0).sleep();
        STATE = 2;    
        break;
      case 2: // do nothing - still moving to the drop off zone.
        break;
      case 3: // got to the drop off zone. add the marker there.
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        STATE = 4; // finished state.   
        break;
      case 4: // finished state.
        break;     
    }
    ros::spinOnce();
  }


    
  ros::spin();
}

