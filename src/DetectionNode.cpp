#include "visualization_msgs/MarkerArray.h"
#include <DetectionNode.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <algorithm>
#include <pluginlib/class_list_macros.h>

namespace artifacts_detection {

/* onInit() method //{ */
    void DetectionNode::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forger to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        int num_of_obj;
        mrs_lib::ParamLoader pl(nh, "DetectionNode");

        pl.loadParam("UAV_NAME", m_uav_name);
        pl.loadParam("human_walking_positions", m_human_positions);
        pl.loadParam("number_of_humans", num_of_obj);
        pl.loadParam("human.offset", m_human_offset);
        pl.loadParam("human.size", m_human_size);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[DetectionNode]: failed to load non-optional parameters!");
            ros::shutdown();
        }
        // As far as 'objects' are as one-dim array, read them carefully
        int counter = 0;
        for(int i = 0; i < num_of_obj; ++i){
            geometry_msgs::Point point_for_reading;
            point_for_reading.x = m_human_positions[counter++];
            point_for_reading.y = m_human_positions[counter++];
            point_for_reading.z = m_human_positions[counter++];
        //    std::cout << point_for_reading << std::endl;
            m_geom_markers.push_back(std::move(point_for_reading));
        }


        m_pub_cube_array = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

        // | --------------------- tf transformer --------------------- |

        m_transformer = mrs_lib::Transformer("DetectionNode");

        // | -------------------- initialize timers ------------------- |

        m_timer_marker = nh.createTimer(ros::Duration(0.1), &DetectionNode::tim_markers_publish, this);

        ROS_INFO_ONCE("[DetectionNode]: initialized");
        is_initialized = true;
    }
//}

// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |
    void DetectionNode::tim_markers_publish([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not is_initialized) return;

        const auto tf_subt_ouster = m_transformer.getTransform("subt", "uav22/os_lidar", ros::Time::now());
        
        if (not tf_subt_ouster.has_value()) {
            ROS_INFO_THROTTLE(1.0, "[DetectionNode] No transformation form %s to %s", "subt", "uav22/os_lidar");
            return;
        }

        const auto transform_stamped = tf_subt_ouster.value();
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "uav22/os_lidar";
        //marker.header.frame_id = "subt";
        marker.ns = "mnspace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        std::vector<geometry_msgs::Point> markers_transformed;
        for (auto &point :m_geom_markers){
            marker.points.push_back(m_transformer.transformHeaderless(transform_stamped, point).value());
            //marker.points.push_back(point);
        }

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 0.4;
        marker.color.g = 1.0;

        marker_array.markers.push_back(marker);

        m_pub_cube_array.publish(marker_array);
        ROS_INFO_THROTTLE(1.0, "[DetectionNode] marker cube array sent");

    }
// | -------------------- other functions ------------------- |

    //}
}  // namespace artifacts_detection  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(artifacts_detection::DetectionNode, nodelet::Nodelet
)
