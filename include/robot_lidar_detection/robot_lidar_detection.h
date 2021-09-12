#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h> // pub, sub images in ROS
#include <cv_bridge/cv_bridge.h> // useful functions for image encodings
#include <sensor_msgs/image_encodings.h> // for ROS-OpenCV conversion: toCvShare, toCvCopy
#include <opencv2/imgproc/imgproc.hpp> // image processing
#include <opencv2/highgui/highgui.hpp> // GUI modules

#include <iostream>

#include <dynamic_reconfigure/server.h>

#include <robot_lidar_detection/RobotLidarDetectionConfig.h>

#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sstream>
#include <limits.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

#include <geometry_msgs/TwistStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#include <visualization_msgs/Marker.h>

#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/pcl_search.h>
#include <pcl/filters/extract_indices.h>

#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef PointCloud::Ptr ptr_cloud;

using namespace cv;
using namespace cv_bridge; // CvImage, toCvShare


namespace robot_lidar_detection
{
  class RobotLidarDetectionNodelet : public nodelet::Nodelet
  {
    public:
      // ROS communication
      boost::shared_ptr<image_transport::ImageTransport> it_in_;
      ros::Subscriber sub_pcl;
      ros::Subscriber sub_odom;

      ros::Publisher pub_output_;
      ros::Publisher pub_pcl_1;
      ros::Publisher pub_pcl_2;
      ros::Publisher pub_dist;
      ros::Publisher pub_estop;
      ros::Publisher pub_slowdown;

      ros::NodeHandle nh;
      ros::NodeHandle private_nh;

      boost::mutex connect_mutex_;

      int queue_size_;
      std::string target_frame_id_;


      // Dynamic reconfigure
      boost::recursive_mutex config_mutex_;
      typedef robot_lidar_detection::RobotLidarDetectionConfig Config;
      typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
      boost::shared_ptr<ReconfigureServer> reconfigure_server_;
      Config config_;

      void onInit();

      // Handles (un)subscribing when clients (un)subscribe;
      void connectCb();

      void configCb(Config &config, uint32_t level);

      void pcl_cb(const sensor_msgs::PointCloud2::ConstPtr&);
      void odom_cb(const nav_msgs::Odometry::ConstPtr&);

    private:

      std::string vehicle_name_;
      float intensity_LT_threshold_;
      float intensity_GT_threshold_;
      float ror_radius_;
      float ror_min_neighbors_;
      float estop_seconds_;
      float slowdown_seconds_;

      bool entrance_flag = 1;
      bool estop_flag = 0;
      bool slowdown_flag = 0;
      int estop_timer = 0;
      int slowdown_timer = 0;
      

  };

};
