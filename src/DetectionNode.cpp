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

        mrs_lib::ParamLoader param_loader(nh, "DetectionNode");
        pl.loadParam("uav_name", m_uav_name);
        pl.loadParam("cube1", m_cube1);
        pl.loadParam("drill1", m_drill1);
        pl.loadParam("survivor1", m_survivor1);

        param_loader.loadParam("UAV_NAME", _uav_name_);

        if (!param_loader.loadedSuccessfully()) {
            ROS_ERROR("[DetectionNode]: failed to load non-optional parameters!");
            ros::shutdown();
        }

        // | --------------------- tf transformer --------------------- |

        transformer_ = mrs_lib::Transformer("DetectionNode", _uav_name_);

        // | -------------------- initialize timers ------------------- |

        ROS_INFO_ONCE("[DetectionNode]: initialized");

        is_initialized = true;
    }
//}

// | ---------------------- msg callbacks --------------------- |


// | --------------------- timer callbacks -------------------- |

// | -------------------- other functions ------------------- |

}  // namespace artifacts_detection  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(artifacts_detection::DetectionNode, nodelet::Nodelet)
