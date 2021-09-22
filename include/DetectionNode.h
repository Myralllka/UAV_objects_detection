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

#include <tf2_eigen/tf2_eigen.h>

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
        std::vector<float> m_cube1;
        std::vector<float> m_drill1;
        std::vector<float> m_survivor1;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_timer_marker;

        void tim_markers_publish([[maybe_unused]] const ros::TimerEvent &ev);
        // | --------- variables, related to message checking --------- |

        // | ----------------------- publishers ----------------------- |

        ros::Publisher m_pub_marker;
        ros::Publisher m_pub_cube1;
        ros::Publisher m_pub_survivor1;
        ros::Publisher m_pub_drill1;

        // | --------------------- other functions -------------------- |

    };
//}

}  // namespace artifacts_detection
