#include <DetectionNode.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
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

        mrs_lib::ParamLoader pl(nh, "DetectionNode");
        pl.loadParam("UAV_NAME", m_uav_name);
        pl.loadParam("cube1", m_cube1);
        pl.loadParam("drill1", m_drill1);
        pl.loadParam("survivor1", m_survivor1);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[DetectionNode]: failed to load non-optional parameters!");
            ros::shutdown();
                       }

        m_pub_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        m_pub_cube1 = nh.advertise<visualization_msgs::Marker>("visualization_cube1", 1);
        m_pub_drill1 = nh.advertise<visualization_msgs::Marker>("visualization_drill1", 1);
        m_pub_survivor1 = nh.advertise<visualization_msgs::Marker>("visualization_survivor1", 1);

        // | --------------------- tf transformer --------------------- |

        m_transformer = mrs_lib::Transformer("DetectionNode", m_uav_name);

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

        visualization_msgs::Marker marker;
        marker.header.frame_id = "subt";
        marker.header.stamp = ros::Time();
        marker.ns = "mnspace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = m_cube1[0];
        marker.pose.position.y = m_cube1[1];
        marker.pose.position.z = m_cube1[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 0.4;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        m_pub_cube1.publish(marker);
        ROS_INFO_THROTTLE(1.0, "[DetectionNode] marker cube sent");

        marker.pose.position.x = m_drill1[0];
        marker.pose.position.y = m_drill1[1];
        marker.pose.position.z = m_drill1[2];
        m_pub_drill1.publish(marker);
        ROS_INFO_THROTTLE(1.0, "[DetectionNode] marker drill sent");


        marker.pose.position.x = m_survivor1[0];
        marker.pose.position.y = m_survivor1[1];
        marker.pose.position.z = m_survivor1[2];
        m_pub_survivor1.publish(marker);
        ROS_INFO_THROTTLE(1.0, "[DetectionNode] marker survivor 1 sent");

    }
// | -------------------- other functions ------------------- |

    //visualization_msgs::Marker create_marker(std::string shape, std::vector<float> position){
        
    //}
}  // namespace artifacts_detection  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(artifacts_detection::DetectionNode, nodelet::Nodelet)
