#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <tf2_eigen/tf2_eigen.h>

/* custom messages */
#include <mrs_detection/Metadata.h>
#include <mrs_detection/MetadataArray.h>

/* point cloud library */
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

//}


namespace artifacts_detection {

/* class DetectionNode //{ */
    class DetectionNode : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool is_initialized = false;
        /* ros parameters */

        std::string m_uav_name;

        std::vector<float> m_human_positions;
        std::vector<float> m_human_offset;
        std::vector<float> m_human_size;

        std::vector<geometry_msgs::Point> m_geom_markers;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        
        void m_callb_pc_processing(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);

        // | --------------------- timer callbacks -------------------- |
        
        ros::Timer m_timer_marker;

        ros::Timer m_timer_obj_positions;

        [[maybe_unused]] void tim_markers_publish([[maybe_unused]] const ros::TimerEvent &ev);

        [[maybe_unused]] void tim_boundbox_write([[maybe_unused]] const ros::TimerEvent &ev);

        // | --------- variables, related to message checking --------- |

        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_cube_array;

        ros::Subscriber m_sub_pc;

        // | --------------------- other functions -------------------- |
    };
//}

}  // namespace artifacts_detection
