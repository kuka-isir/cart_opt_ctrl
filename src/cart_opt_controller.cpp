#include <cart_opt_ctrl/cart_opt_controller.hpp>
#include <pluginlib/class_list_macros.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace cart_opt_controllers
{
  /**
   * \brief Joint trajectory controller that represents trajectory segments as <b>quintic splines</b> and sends
   * commands to an CartOptEffortJointInterface
   */
  typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::CartOptEffortJointInterface>
          JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(cart_opt_controllers::JointTrajectoryController, controller_interface::ControllerBase)