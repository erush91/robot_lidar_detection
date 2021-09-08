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
        private_nh.param<std::string>("vehicle_name", vehicle_name_,"H02");

        // Set up dynamic reconfigure
        reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
        ReconfigureServer::CallbackType f = boost::bind(&RobotLidarDetectionNodelet::configCb, this, _1, _2);
        reconfigure_server_->setCallback(f);

        sub_pcl = nh.subscribe("pcl_in", 1, &RobotLidarDetectionNodelet::pcl_cb, this);
        

        // Publish Point Cloud
        pub_pcl_1    = private_nh.advertise<PointCloud>("cloud_hi_intensity/raw", 10);
        pub_pcl_2    = private_nh.advertise<PointCloud>("cloud_hi_intensity/filtered", 10);
        pub_dist     = private_nh.advertise<std_msgs::Float32>("robot_distance", 10);
        pub_estop    = private_nh.advertise<std_msgs::Bool>("estop", 10);
        pub_slowdown = private_nh.advertise<std_msgs::Bool>("slowdown", 10);
    };

    void RobotLidarDetectionNodelet::configCb(Config &config, uint32_t level)
    {
        config_ = config;

        intensity_LT_threshold_     = config.intensity_LT_threshold;
        intensity_GT_threshold_     = config.intensity_GT_threshold;
        ror_radius_                 = config.ror_radius;
        ror_min_neighbors_          = config.ror_min_neighbors;
        estop_seconds_              = config.estop_seconds;
        slowdown_seconds_           = config.slowdown_seconds;
    };

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

        if(cloud_hi_intensity->points.size() > 0)
        {
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
        }

        float distance_min;
        float distance_sq;
        float distance_sq_last;
        float distance_sq_min;
        float traveled_distance;
        // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)

        if(cloud_ror->points.size() > 0)
        {
            for (int i = 0; i < cloud_ror->points.size(); i++)
            {
                distance_sq = cloud_ror->points[i].x * cloud_ror->points[i].x
                            + cloud_ror->points[i].y * cloud_ror->points[i].y
                            + cloud_ror->points[i].z * cloud_ror->points[i].z;
                distance_sq_last = distance_sq;
                distance_sq_min = min(distance_sq, distance_sq_last);
            }
            
            distance_min = sqrt(distance_sq_min);

            if(distance_min != 0)
            {
                std_msgs::Float32 robot_distance;
                robot_distance.data = distance_min;
                pub_dist.publish(robot_distance);
            }
        }

        if(distance_min != 0 && distance_min < 2.0 && !estop_flag && !slowdown_flag)
        {
            estop_flag = true;
            slowdown_flag = false;

            std_msgs::Bool estop;
            estop.data = true;
            pub_estop.publish(estop);
        }


        if(estop_timer >= 20 * estop_seconds_)
        {
            estop_flag = false;
            slowdown_flag = true;
            estop_timer = 0;

            std_msgs::Bool estop;
            estop.data = false;
            pub_estop.publish(estop);

            std_msgs::Bool slowdown;
            slowdown.data = true;
            pub_slowdown.publish(slowdown);
        }

        if(slowdown_timer >= 20 * slowdown_seconds_)
        {
            estop_flag = false;
            slowdown_flag = false;
            slowdown_timer = 0;
            
            std_msgs::Bool slowdown;
            slowdown.data = false;
            pub_slowdown.publish(slowdown);
        }

        if(estop_flag)
        {
            estop_timer++;
        }

        if(slowdown_flag)
        {
            slowdown_timer++;
        }

        ROS_INFO("DISTANCE [m]: %.3f // ESTOP %i  %i // SLOWDOWN %i  %i", distance_min, estop_flag, estop_timer, slowdown_flag, slowdown_timer);
        // std::cout << estop_flag << " / " << slowdown_flag << " / " << estop_timer << " / " << slowdown_timer ...
        //           << " / " traveled_distance ...
        //           << " / " << odom_x << " / " << odom_y << " / "  << " / " <<  ...
        //           << " / " << odom_estop_x << " / " << odom_estop_u << " / " << " / " std::endl;
        
    };
}


// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( robot_lidar_detection::RobotLidarDetectionNodelet, nodelet::Nodelet)
