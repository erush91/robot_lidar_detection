#include <robot_lidar_detection/robot_lidar_detection.h>

namespace robot_lidar_detection
{
    void RobotLidarDetectionNodelet::connectCb()
    {
    };

    void RobotLidarDetectionNodelet::onInit()
    {
        nh         = getMTNodeHandle();
        private_nh = getMTPrivateNodeHandle();

        // Read parameters
        private_nh.param("queue_size", queue_size_, 5);
        private_nh.param("target_frame_id", target_frame_id_, std::string());

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
        ReconfigureServer::CallbackType f = boost::bind(&RobotLidarDetectionNodelet::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);

        sub_pcl = nh.subscribe("pcl_in", 1, &RobotLidarDetectionNodelet::pcl_cb, this);
        sub_odom = nh.subscribe("odom_in", 1, &RobotLidarDetectionNodelet::odom_cb, this);

        // Publish Point Cloud
        pub_pcl_1    = private_nh.advertise<PointCloud>("pcl_out_1", 10);
        pub_pcl_2    = private_nh.advertise<PointCloud>("pcl_out_2", 10);
        pub_dist     = private_nh.advertise<std_msgs::Float32>("robot_distance", 10);
        pub_estop    = private_nh.advertise<std_msgs::Bool>("estop", 10);
    };

    void RobotLidarDetectionNodelet::configCb(Config &config, uint32_t level)
    {
        config_ = config;

        intensity_LT_threshold_     = config.intensity_LT_threshold;
        intensity_GT_threshold_     = config.intensity_GT_threshold;
        ror_radius_                 = config.ror_radius;
        ror_min_neighbors_          = config.ror_min_neighbors;
        estop_seconds_              = config.estop_seconds;
        estop_timeout_seconds_      = config.estop_timeout_seconds;
        estop_reset_radius_         = config.estop_reset_radius;

    };


    void RobotLidarDetectionNodelet::odom_cb(const nav_msgs::Odometry::ConstPtr& odom_in)
    {
        odom_x = odom_in->pose.pose.position.x;
        odom_y = odom_in->pose.pose.position.y;
        odom_z = odom_in->pose.pose.position.z;
    }

    void RobotLidarDetectionNodelet::pcl_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_in_ros)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_in_ros, *cloud_in);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hi_intensity (new pcl::PointCloud<pcl::PointXYZI>);
        
        // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            if(cloud_in->points[i].intensity > intensity_GT_threshold_ && cloud_in->points[i].intensity < intensity_LT_threshold_)
                cloud_hi_intensity->push_back(cloud_in->points[i]);
        }

        cloud_hi_intensity->header.seq = cloud_in->header.seq;
        cloud_hi_intensity->header.frame_id = cloud_in->header.frame_id;
        pcl_conversions::toPCL(ros::Time::now(), cloud_hi_intensity->header.stamp);
        pub_pcl_1.publish (cloud_hi_intensity);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ror(new pcl::PointCloud<pcl::PointXYZI>);

        // tuning for detecting robots at 1.5m away
        //float ror_ror_radius_ = 0.05;
        //float ror_ror_min_neighbors_ = 40;

        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(cloud_hi_intensity);
        outrem.setRadiusSearch(ror_radius_);
        outrem.setMinNeighborsInRadius(ror_min_neighbors_);
        // outrem.setKeepOrganized(true);
        outrem.filter (*cloud_ror);

        cloud_ror->header.seq = cloud_in->header.seq;
        cloud_ror->header.frame_id = cloud_in->header.frame_id;
        pcl_conversions::toPCL(ros::Time::now(), cloud_ror->header.stamp);
        pub_pcl_2.publish (cloud_ror);

        // distance_min = 0;
        // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)

        for (int i = 0; i < cloud_ror->points.size(); i++)
        {
            distance_sq = cloud_ror->points[i].x * cloud_ror->points[i].x
                        + cloud_ror->points[i].y * cloud_ror->points[i].y
                        + cloud_ror->points[i].z * cloud_ror->points[i].z;
            distance_sq_last = distance_sq;
            distance_sq_min = min(distance_sq, distance_sq_last);
        }

        distance_min = sqrt(distance_sq_min);
        
        if(distance_min > 0.5) // ignore if distance_min = 0 (that means nothing was detected)
        {
            std_msgs::Float32 robot_distance;
            robot_distance.data = distance_min;
            pub_dist.publish(robot_distance);
        }

        if(distance_min > 0.5 && distance_min < 1.0 && !estop_flag && !estop_timeout_flag)
        {
            estop_flag = true;
            std_msgs::Bool estop;
            estop.data = true;
            pub_estop.publish(estop);
            odom_estop_x = odom_x;
            odom_estop_y = odom_y;
            odom_estop_z = odom_z;
        }

        if(odom_estop_x > 0) // only compute if estop has been triggered at least once
        {
            traveled_distance = sqrt((odom_estop_x - odom_x) * (odom_estop_x - odom_x)
                                   + (odom_estop_y - odom_y) * (odom_estop_y - odom_y)
                                   + (odom_estop_z - odom_z) * (odom_estop_z - odom_z));
        }

        if(estop_flag)
        {
            estop_timer++;
        }

        if(estop_timeout_flag)
        {
            estop_timeout_timer++;
        }

        if(estop_timer > 20 * estop_seconds_)
        {
            estop_flag = false;
            estop_timeout_flag = true;
            estop_timer = 0;
            std_msgs::Bool estop;
            estop.data = false;
            pub_estop.publish(estop);
        }

        if(estop_timeout_timer > 20 * estop_timeout_seconds_ && traveled_distance > estop_reset_radius_)
        {
            estop_flag = false;
            estop_timeout_flag = false;
            estop_timeout_timer = 0;
        }

        ROS_INFO("%.2f / %i / %i / %i / %i / %.1f / %.1f / %.1f / %.1f / %.1f / %.1f / %.1f", distance_min, estop_flag, estop_timeout_flag, estop_timer, estop_timeout_timer, traveled_distance, odom_x, odom_y, odom_z, odom_estop_x, odom_estop_y, odom_estop_z);
        // std::cout << estop_flag << " / " << estop_timeout_flag << " / " << estop_timer << " / " << estop_timeout_timer ...
        //           << " / " traveled_distance ...
        //           << " / " << odom_x << " / " << odom_y << " / " << odom_z << " / " <<  ...
        //           << " / " << odom_estop_x << " / " << odom_estop_u << " / " << odom_estop_z << " / " std::endl;
        
    };
}


// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( robot_lidar_detection::RobotLidarDetectionNodelet, nodelet::Nodelet)
