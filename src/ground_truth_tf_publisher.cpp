// clang: MatousFormat
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ground_truth_tf_publisher");
	ros::NodeHandle nh = ros::NodeHandle("~");

  mrs_lib::ParamLoader pl(nh);
  double rate = pl.load_param2<double>("ground_truth/tf_publish_rate", 50.0);
  string gt_frame_id = pl.load_param2<string>("ground_truth/ground_truth_frame_id");
  string other_frame_id = pl.load_param2<string>("ground_truth/other_frame_id");
  string local_frame_id = pl.load_param2<string>("ground_truth/local_frame_id");

  if (!pl.loaded_successfully())
  {
    ROS_ERROR("[%s]: Could not load all compulsory parameters", ros::this_node::getName().c_str());
    ros::shutdown();
  }

  mrs_lib::SubscribeMgr smgr(nh);
  auto sh_gt_odom = smgr.create_handler<nav_msgs::Odometry>("ground_truth_odom", ros::Duration(5.0));

  tf2_ros::TransformBroadcaster tf2_bc;
  tf2_ros::Buffer tf2_bfr;
  tf2_ros::TransformListener tf2_listener(tf2_bfr, nh);

  ros::Rate r(rate);
  while (ros::ok())
  {
    ros::spinOnce();

    if (sh_gt_odom->new_data())
    {
      nav_msgs::Odometry gt_odom = sh_gt_odom->get_data();
      geometry_msgs::Transform gt2world_tf_ros;
      gt2world_tf_ros.rotation = gt_odom.pose.pose.orientation;
      gt2world_tf_ros.translation.x = gt_odom.pose.pose.position.x;
      gt2world_tf_ros.translation.y = gt_odom.pose.pose.position.y;
      gt2world_tf_ros.translation.z = gt_odom.pose.pose.position.z;
      /* cout << "Ground truth: " << endl << gt2world_tf_ros << endl; */
      Eigen::Affine3d gt2world_tf = tf2::transformToEigen(gt2world_tf_ros);

      try
      {
        const ros::Duration timeout(1.0 / 10.0);
        // Obtain transform from snesor into world frame
        geometry_msgs::TransformStamped world2other_tf_ros = tf2_bfr.lookupTransform(other_frame_id, local_frame_id, gt_odom.header.stamp, timeout);
        /* cout << "Odometry: " << endl << other2world_tf_ros << endl; */
        Eigen::Affine3d world2other_tf = tf2::transformToEigen(world2other_tf_ros);

        Eigen::Affine3d lo2gt = gt2world_tf*world2other_tf;

        geometry_msgs::TransformStamped to_publish_tf = tf2::eigenToTransform(lo2gt);
        to_publish_tf.header.stamp = gt_odom.header.stamp;
        to_publish_tf.header.frame_id = gt_frame_id;
        to_publish_tf.child_frame_id = local_frame_id;

        /* cout << "GT to odom: " << endl << to_publish_tf << endl; */
        tf2_bc.sendTransform(to_publish_tf);

      } catch (tf2::TransformException& ex)
      {
        ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", other_frame_id.c_str(), local_frame_id.c_str(), ex.what());
      }
    }

    r.sleep();
  }

	ros::shutdown();
}

