#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

sensor_msgs::JointState joint_states;

void js_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    for(int k = 0; k < 18; k++)
    {
        joint_states.name[k] = msg->name[k];
        joint_states.position[k] = msg->position[k];
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber sub = node.subscribe("kautham_joint_states", 1, js_cb);

  joint_states.name.resize(18);
  joint_states.name[0]="yumi_joint_1_l";
  joint_states.name[1]="yumi_joint_2_l";
  joint_states.name[2]="yumi_joint_7_l";
  joint_states.name[3]="yumi_joint_3_l";
  joint_states.name[4]="yumi_joint_4_l";
  joint_states.name[5]="yumi_joint_5_l";
  joint_states.name[6]="yumi_joint_6_l";

  joint_states.name[8]="yumi_joint_1_r";
  joint_states.name[9]="yumi_joint_2_r";
  joint_states.name[10]="yumi_joint_7_r";
  joint_states.name[11]="yumi_joint_3_r";
  joint_states.name[12]="yumi_joint_4_r";
  joint_states.name[13]="yumi_joint_5_r";
  joint_states.name[14]="yumi_joint_6_r";

  joint_states.name[7]="gripper_l_joint";
  joint_states.name[15]="gripper_r_joint";
  joint_states.name[16]="gripper_l_joint_m";
  joint_states.name[17]="gripper_r_joint_m";
  joint_states.position.resize(18, 0);
  joint_states.position[0] = 1.1;
  joint_states.position[8] = -1.1;
  joint_states.position[7] = 0.025;
  joint_states.position[15] = 0.025;
  joint_states.position[16] = 0.025;
  joint_states.position[17] = 0.025;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = "map";
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;
  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;
  
  ros::Rate rate(100);
  while (node.ok()){
    ros::spinOnce();
    static_broadcaster.sendTransform(static_transformStamped);
    joint_states.header.stamp = ros::Time::now();
    joint_pub.publish(joint_states);
    rate.sleep();
  }
  ros::spin();
  return 0;
}; 
