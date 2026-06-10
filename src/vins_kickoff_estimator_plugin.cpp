/* includes //{ */

/* #include <pairs_vins_kickoff_estimator_plugin/include/vins_kickoff.h> */
#include "../include/vins_kickoff_estimator_plugin.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

//}

namespace vins_kickoff
{

/* initialize() //{*/
void VinsKickoff::initialize([[maybe_unused]] const rclcpp::Node::SharedPtr &node,
                             const std::shared_ptr<pairs_uav_managers::estimation_manager::CommonHandlers_t> &ch,
                             const std::shared_ptr<pairs_uav_managers::estimation_manager::PrivateHandlers_t> &ph) {

  ch_ = ch;
  ph_ = ph;

  ns_frame_id_ = ch_->uav_name + "/" + frame_id_;

  // | --------------- param loader initialization -------------- |

  ph->param_loader->addYamlFile(ament_index_cpp::get_package_share_directory("pairs_uav_managers") + "/config/public/uav_manager.yaml");

  ph->param_loader->loadParam("pairs_uav_managers/uav_manager/takeoff/during_takeoff/tracker", takeoff_tracker_name_);
  /* takeoff_tracker_name_ = "LandoffTracker"; */

  ph->param_loader->setPrefix(ch_->package_name + "/" + Support::toSnakeCase(ch_->nodelet_name) + "/" + getName() + "/");

  // | --------------------- load parameters -------------------- |

  double max_kicking_time;
  ph->param_loader->loadParam("kickoff/max_duration", max_kicking_time);
  dur_max_kicking_ = rclcpp::Duration(std::chrono::seconds(static_cast<int>(max_kicking_time)));

  ph->param_loader->loadParam("kickoff/target_estimator", target_estimator_);

  std::string topic_orientation;
  ph->param_loader->loadParam("topics/orientation", topic_orientation);
  topic_orientation_ = "/" + ch_->uav_name + "/" + topic_orientation;
  std::string topic_angular_velocity;
  ph->param_loader->loadParam("topics/angular_velocity", topic_angular_velocity);
  topic_angular_velocity_ = "/" + ch_->uav_name + "/" + topic_angular_velocity;

  // | ------------------ timers initialization ----------------- |
  //
  timer_update_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / ch_->desired_uav_state_rate),
    std::bind(&VinsKickoff::timerUpdate, this)
  );

  // | ---------------- publishers initialization --------------- |
  ph_odom_ = pairs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, Support::toSnakeCase(getName()) + "/odom");

  if (ch_->debug_topics.state) {
    ph_uav_state_ = pairs_lib::PublisherHandler<pairs_msgs::msg::UavState>(node_, Support::toSnakeCase(getName()) + "/uav_state");
  }
  if (ch_->debug_topics.covariance) {
    ph_pose_covariance_  = pairs_lib::PublisherHandler<pairs_msgs::msg::Float64ArrayStamped>(node_, Support::toSnakeCase(getName()) + "/pose_covariance");
    ph_twist_covariance_ = pairs_lib::PublisherHandler<pairs_msgs::msg::Float64ArrayStamped>(node_, Support::toSnakeCase(getName()) + "/twist_covariance");
  }
  if (ch_->debug_topics.innovation) {
    ph_innovation_ = pairs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, Support::toSnakeCase(getName()) + "/innovation");
  }
  if (ch_->debug_topics.diag) {
    ph_diagnostics_ = pairs_lib::PublisherHandler<pairs_msgs::msg::EstimatorDiagnostics>(node_, Support::toSnakeCase(getName()) + "/diagnostics");
  }

  // | --------------- subscribers initialization --------------- |
  pairs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = node_;
  shopts.node_name          = getPrintName();
  shopts.no_message_timeout = pairs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  //shopts.qos                = rclcpp::QoS(rclcpp::KeepLast(10));
  //shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_manager_diag_    = pairs_lib::SubscriberHandler<pairs_msgs::msg::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_controller_diag_         = pairs_lib::SubscriberHandler<pairs_msgs::msg::ControllerDiagnostics>(shopts, "controller_diagnostics_in");
  sh_estimation_manager_diag_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::EstimationDiagnostics>(shopts, "diagnostics_out");
  sh_hw_api_orient_           = pairs_lib::SubscriberHandler<geometry_msgs::msg::QuaternionStamped>(shopts, topic_orientation_);
  sh_hw_api_ang_vel_          = pairs_lib::SubscriberHandler<geometry_msgs::msg::Vector3Stamped>(shopts, topic_angular_velocity_);
  sh_control_reference_       = pairs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "control_reference_in");

  /*//{ initialize service clients */

  // | ------------- service clients initialization ------------- |
  srvch_failsafe_ = pairs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "failsafe_out");
  srvch_switch_estimator_ = pairs_lib::ServiceClientHandler<pairs_msgs::srv::String>(node_, "change_estimator_in");

  /*//}*/

  // | ------------------ initialize published messages ------------------ |
  uav_state_init_.header.frame_id = ns_frame_id_;
  uav_state_init_.child_frame_id  = ch_->frames.ns_fcu;

  uav_state_init_.estimator_horizontal = est_lat_name_;
  uav_state_init_.estimator_vertical   = est_alt_name_;
  uav_state_init_.estimator_heading    = est_hdg_name_;

  uav_state_init_.pose.position.x = 0.0;
  uav_state_init_.pose.position.y = 0.0;
  uav_state_init_.pose.position.z = 0.0;

  uav_state_init_.pose.orientation.x = 0.0;
  uav_state_init_.pose.orientation.y = 0.0;
  uav_state_init_.pose.orientation.z = 0.0;
  uav_state_init_.pose.orientation.w = 1.0;

  uav_state_init_.velocity.linear.x = 0.0;
  uav_state_init_.velocity.linear.y = 0.0;
  uav_state_init_.velocity.linear.z = 0.0;

  uav_state_init_.velocity.angular.x = 0.0;
  uav_state_init_.velocity.angular.y = 0.0;
  uav_state_init_.velocity.angular.z = 0.0;

  innovation_init_.header.frame_id         = ns_frame_id_;
  innovation_init_.child_frame_id          = ch_->frames.ns_fcu;
  innovation_init_.pose.pose.orientation.w = 1.0;

  // | ------------------ finish initialization ----------------- |

  if (changeState(INITIALIZED_STATE)) {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator initialized", getPrintName().c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator could not be initialized", getPrintName().c_str());
  }
}
/*//}*/

/*//{ start() */
bool VinsKickoff::start(void) {


  if (isInState(READY_STATE)) {

    changeState(STARTED_STATE);
    return true;

  } else {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator must be in READY_STATE to start it", getPrintName().c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  return false;

  RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to start", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ pause() */
bool VinsKickoff::pause(void) {

  if (isInState(RUNNING_STATE)) {
    changeState(STOPPED_STATE);
    return true;

  } else {
    return false;
  }
}
/*//}*/

/*//{ reset() */
bool VinsKickoff::reset(void) {

  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s]: Cannot reset uninitialized estimator", getPrintName().c_str());
    return false;
  }

  changeState(STOPPED_STATE);

  RCLCPP_INFO(node_->get_logger(), "[%s]: Estimator reset", getPrintName().c_str());

  return true;
}
/*//}*/

/* timerUpdate() //{*/
void VinsKickoff::timerUpdate() {


  if (!isInitialized()) {
    return;
  }

  switch (getCurrentSmState()) {

    case UNINITIALIZED_STATE: {
      break;
    }
    case INITIALIZED_STATE: {

      if (sh_hw_api_orient_.hasMsg() && sh_hw_api_ang_vel_.hasMsg()) {
        changeState(READY_STATE);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: Estimator is ready to start", getPrintName().c_str());
      } else {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: %s msg on topic %s", getPrintName().c_str(), Support::waiting_for_string.c_str(), sh_hw_api_orient_.topicName().c_str());
        return;
      }

      break;
    }

    case READY_STATE: {
      break;
    }

    case STARTED_STATE: {

      if (!sh_control_manager_diag_.hasMsg()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for control manager diagnostics", getPrintName().c_str());
        return;
      }

      changeState(RUNNING_STATE);

      break;
    }

    case RUNNING_STATE: {

      // first we wait until we start taking off (detected by switching into landoff tracker and finishing rampup)
      if (!is_taking_off_ && sh_control_manager_diag_.hasMsg() && sh_control_manager_diag_.getMsg()->active_tracker == takeoff_tracker_name_ &&
          sh_controller_diag_.hasMsg() && !sh_controller_diag_.getMsg()->ramping_up) {
        t_init_kickoff_ = node_->now();
        is_taking_off_  = true;
      }

      if (is_taking_off_) {

        RCLCPP_INFO(node_->get_logger(), "[%s]: time kicking off: %.2f/%.2f", getPrintName().c_str(), (node_->now() - t_init_kickoff_).seconds(), dur_max_kicking_.seconds());

        // we are waiting until vins estimator is initialized so we can switch into it
        if (isVinsEstimatorInitialized()) {
          RCLCPP_INFO(node_->get_logger(), "[%s]: %s estimator is running, trying to switch", getPrintName().c_str(), target_estimator_.c_str());
          switch_estimator_call_succeeded_ = callSwitchEstimatorService();
        }

        // switch into vins estimator successful, this kickoff estimator's job is done
        if (switch_estimator_call_succeeded_) {
          RCLCPP_INFO(node_->get_logger(), "[%s]: vins kickoff took %.2f", getPrintName().c_str(), (node_->now() - t_init_kickoff_).seconds());
          changeState(STOPPED_STATE);
        }

        // did not manage to initialize vins estimator in time, call failsafe
        if (node_->now() - t_init_kickoff_ > dur_max_kicking_) {
          RCLCPP_ERROR(node_->get_logger(), "[%s]: max kickoff time elapsed without vins estimator initialization, calling failsafe", getPrintName().c_str());
          failsafe_call_succeeded_ = callFailsafeService();
          changeState(ERROR_STATE);
        }
      }

      break;
    }

    case STOPPED_STATE: {
      RCLCPP_INFO_ONCE(node_->get_logger(), "[%s]: my job is done here", getPrintName().c_str());
      break;
    }

    case ERROR_STATE: {
      // call failsafe until the call succeeds
      if (!failsafe_call_succeeded_) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: calling failsafe", getPrintName().c_str());
        failsafe_call_succeeded_ = callFailsafeService();
      }
      break;
    }
  }

  if (!isRunning() && !isStarted()) {
    return;
  }

  updateUavState();

  publishUavState();
  publishOdom();
  publishCovariance();
  publishInnovation();
  publishDiagnostics();
}  // namespace vins_kickoff
/*//}*/

/*//{ updateUavState() */
void VinsKickoff::updateUavState() {

  if (!sh_hw_api_orient_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received orientation on topic %s yet", getPrintName().c_str(), sh_hw_api_orient_.topicName().c_str());
    return;
  }

  if (!sh_hw_api_ang_vel_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: has not received angular velocity on topic %s yet", getPrintName().c_str(), sh_hw_api_ang_vel_.topicName().c_str());
    return;
  }

  pairs_lib::ScopeTimer scope_timer = pairs_lib::ScopeTimer(node_, "StateGeneric::updateUavState", ch_->scope_timer.logger, ch_->scope_timer.enabled);

  const rclcpp::Time time_now = node_->now();

  pairs_msgs::msg::UavState uav_state = uav_state_init_;
  uav_state.header.stamp            = time_now;

  const double hdg = 0;

  // we want to have attitude feedback with zero heading
  auto res = rotateQuaternionByHeading(sh_hw_api_orient_.getMsg()->quaternion, hdg);
  if (res) {
    uav_state.pose.orientation = res.value();
  } else {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: could not rotate orientation by heading", getPrintName().c_str());
    return;
  }

  // we want to have angular velocity feedback
  uav_state.velocity.angular = sh_hw_api_ang_vel_.getMsg()->vector;

  // we want to set the current position slightly below the reference
  if (sh_control_reference_.hasMsg()) {
    uav_state.pose.position   = sh_control_reference_.getMsg()->pose.pose.position;
    uav_state.velocity.linear = sh_control_reference_.getMsg()->twist.twist.linear;
    uav_state.pose.position.z -= 0.2;
    RCLCPP_INFO(node_->get_logger(), "[%s]: fabricated z: %.2f", getPrintName().c_str(), uav_state.pose.position.z);
  }

  const nav_msgs::msg::Odometry odom = Support::uavStateToOdom(uav_state);

  nav_msgs::msg::Odometry innovation = innovation_init_;

  innovation.header.stamp = time_now;

  innovation.pose.pose.position.x = 0.0;
  innovation.pose.pose.position.y = 0.0;
  innovation.pose.pose.position.z = 0.0;

  pairs_msgs::msg::Float64ArrayStamped pose_covariance, twist_covariance;
  pose_covariance_.header.stamp  = time_now;
  twist_covariance_.header.stamp = time_now;

  const int n_states = 6;  // TODO this should be defined somewhere else
  pose_covariance.values.resize(n_states * n_states);
  twist_covariance.values.resize(n_states * n_states);

  pairs_lib::set_mutexed(mtx_uav_state_, uav_state, uav_state_);
  pairs_lib::set_mutexed(mtx_odom_, odom, odom_);
  pairs_lib::set_mutexed(mtx_innovation_, innovation, innovation_);
  pairs_lib::set_mutexed(mtx_covariance_, pose_covariance, pose_covariance_);
  pairs_lib::set_mutexed(mtx_covariance_, twist_covariance, twist_covariance_);
}
/*//}*/

/*//{ isVinsEstimatorInitialized() */
bool VinsKickoff::isVinsEstimatorInitialized() {

  const auto estimators = sh_estimation_manager_diag_.getMsg()->switchable_state_estimators;

  // check whether the vins estimator is ready to be switched into
  return std::find(estimators.begin(), estimators.end(), target_estimator_) != estimators.end();
}
/*//}*/

/*//{ setUavState() */
bool VinsKickoff::setUavState([[maybe_unused]] const pairs_msgs::msg::UavState &uav_state) {

  if (!isInState(STOPPED_STATE)) {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Estimator state can be set only in the STOPPED state", getPrintName().c_str());
    return false;
  }

  RCLCPP_WARN(node_->get_logger(), "[%s]: Setting the state of this estimator is not implemented.", getPrintName().c_str());
  return false;
}
/*//}*/

/*//{ callFailsafeService() */
bool VinsKickoff::callFailsafeService() {
  auto srv_in = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto srv_out = srvch_failsafe_.callSync(srv_in).value();
  return srv_out->success;
}
/*//}*/

/*//{ callSwitchEstimatorService() */
bool VinsKickoff::callSwitchEstimatorService() {
  auto srv_in = std::make_shared<pairs_msgs::srv::String::Request>();
  srv_in->value = target_estimator_;

  auto srv_out = srvch_switch_estimator_.callSync(srv_in).value();
  return srv_out->success;
}
/*//}*/

}  // namespace vins_kickoff

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vins_kickoff::VinsKickoff);
