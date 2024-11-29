/**
 *
 * Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
 *
 */

#include "common.h"

using namespace std;

class ImageGrabber
{
public:
  ImageGrabber(){};

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Mono");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  if (argc > 1)
  {
    ROS_WARN("Arguments supplied via command line are ignored.");
  }

  std::string node_name = ros::this_node::getName();

  ros::NodeHandle node_handler;
  image_transport::ImageTransport image_transport(node_handler);

  std::string voc_file, settings_file;
  node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
  node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

  if (voc_file == "file_not_set" || settings_file == "file_not_set")
  {
    ROS_ERROR("Please provide voc_file and settings_file in the launch file");
    ros::shutdown();
    return 1;
  }

  node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
  node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

  bool enable_pangolin;
  node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

  string traj_file_name, map_file_name, timing_file_name;
  bool do_save_traj, do_save_map, do_loop_closing;
  node_handler.param<bool>(node_name + "/save_traj", do_save_traj, true);
  node_handler.param<bool>(node_name + "/save_map", do_save_map, true);
  node_handler.param<bool>(node_name + "/do_loop_closing", do_loop_closing, true);
  node_handler.param<string>(node_name + "/traj_file_name", traj_file_name, "traj_file_name");
  node_handler.param<string>(node_name + "/map_file_name", map_file_name, "map_file_name");
  node_handler.param<string>(node_name + "/timing_file_name", timing_file_name, "timing_file_name");

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  sensor_type = ORB_SLAM3::System::MONOCULAR;
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin, do_loop_closing);
  ImageGrabber igb;

  ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

  setup_publishers(node_handler, image_transport, node_name);
  // setup_services(node_handler, node_name);

  static bool termination_requested = false;
  std::signal(SIGINT, [](int signal) { termination_requested = true; });

  auto last_save_time = std::chrono::system_clock::now();
  // The initial save interval is higher because if we try to save too early
  // the algorithm crashes.
  auto save_interval = 60.0f;

  while (!ros::isShuttingDown() && !termination_requested)
  {
    ros::spinOnce();

    auto current_time = std::chrono::system_clock::now();
    auto elapsed_time = std::chrono::duration<float>(current_time - last_save_time);

    if (elapsed_time.count() >= save_interval)
    {
      save_traj(traj_file_name);
      last_save_time = current_time;
      save_interval = 5.0f;
    }
  }

  // Stop all threads
  pSLAM->Shutdown();

  pSLAM->SaveTimingAnalysis(timing_file_name);

  return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // ORB-SLAM3 runs in TrackMonocular()
  Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());

  ros::Time msg_time = msg->header.stamp;

  publish_topics(msg_time);
}