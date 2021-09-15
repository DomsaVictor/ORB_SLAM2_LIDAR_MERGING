#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  lidar_subscriber = node_handle.subscribe("/base_scan", 1, &MonoNode::LidarCallback, this);
  camera_info_topic_ = "/camera/camera_info";
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update ();
}

void MonoNode::LidarCallback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int m = (int) (msg->angle_max - msg->angle_min) / msg->angle_increment;
  float current_angle = 0.0f;
  cv::Mat points_x, points_y;
  for (int i=0; i<m; i++)
  {
    if (current_angle <= 31 || current_angle >= 329)
    {
      points_x.push_back(std::sin(current_angle) * msg->ranges[i]);
      points_y.push_back(std::cos(current_angle) * msg->ranges[i]);
    }
  }
  orb_slam_->GetTracker()->SetLidarPoints(points_x, points_y);
}