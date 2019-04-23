#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

// publisher for publishing transformations between world and robot base frame
static tf::TransformBroadcaster tf_broadcast;
// transformation data in quaternion
tf::Transform transform;
tf::Quaternion q;

void pose_callback(const geometry_msgs::PoseStampedPtr &pose)
{

    q.setX(pose->pose.orientation.x);
    q.setY(pose->pose.orientation.y);
    q.setZ(pose->pose.orientation.z);
    q.setW(pose->pose.orientation.w);

    transform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0));
    transform.setRotation(q);
    // publish transformation bw static odom frame and moving robot base frame with timestamp
    tf_broadcast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle n("~");
    ros::Subscriber pose_sub = n.subscribe("/pose", 1, pose_callback);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
}
