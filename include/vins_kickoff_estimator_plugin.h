#ifndef ESTIMATORS_STATE_VINS_KICKOFF_H
#define ESTIMATORS_STATE_VINS_KICKOFF_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <pairs_msgs/ControlManagerDiagnostics.h>
#include <pairs_msgs/ControllerDiagnostics.h>
#include <pairs_msgs/EstimationDiagnostics.h>
#include <pairs_msgs/String.h>

#include <std_srvs/Trigger.h>

#include <pairs_lib/param_loader.h>
#include <pairs_lib/subscribe_handler.h>
#include <pairs_lib/publisher_handler.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/transformer.h>
#include <pairs_lib/service_client_handler.h>

#include <pairs_uav_managers/state_estimator.h>

//}

using namespace pairs_uav_managers::estimation_manager;

namespace vins_kickoff
{

const char name[]         = "vins_kickoff";
const char frame_id[]     = "vins_kickoff_origin";
const char package_name[] = "pairs_uav_state_estimators";

/* using CommonHandlers_t  = pairs_uav_managers::estimation_manager::CommonHandlers_t; */
/* using PrivateHandlers_t = pairs_uav_managers::estimation_manager::PrivateHandlers_t; */

class VinsKickoff : public pairs_uav_managers::StateEstimator {

private:
  const std::string package_name_ = "pairs_uav_state_estimators";

  const std::string est_lat_name_ = "lat_vins_kickoff";

  const std::string est_alt_name_ = "alt_vins_kickoff";

  const std::string est_hdg_name_ = "hdg_vins_kickoff";

  ros::Timer timer_update_;
  void       timerUpdate(const ros::TimerEvent &event);
  bool       first_iter_ = true;

  pairs_lib::SubscribeHandler<pairs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  std::string                                                    takeoff_tracker_name_;

  pairs_lib::SubscribeHandler<nav_msgs::Odometry> sh_control_reference_;

  pairs_lib::SubscribeHandler<pairs_msgs::ControllerDiagnostics> sh_controller_diag_;

  pairs_lib::SubscribeHandler<pairs_msgs::EstimationDiagnostics> sh_estimation_manager_diag_;

  pairs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_hw_api_orient_;
  std::string                                                 topic_orientation_;

  pairs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped> sh_hw_api_ang_vel_;
  std::string                                              topic_angular_velocity_;

  bool                                             callFailsafeService();
  pairs_lib::ServiceClientHandler<std_srvs::Trigger> srvch_failsafe_;
  bool                                             failsafe_call_succeeded_ = false;

  bool                                            callSwitchEstimatorService();
  pairs_lib::ServiceClientHandler<pairs_msgs::String> srvch_switch_estimator_;
  bool                                            switch_estimator_call_succeeded_ = false;

  ros::Duration dur_max_kicking_;
  ros::Time     t_init_kickoff_;
  std::string   target_estimator_;
  bool          is_taking_off_ = false;

  void updateUavState();

  bool isVinsEstimatorInitialized();

public:
  VinsKickoff() : StateEstimator(vins_kickoff::name, vins_kickoff::frame_id, vins_kickoff::package_name) {
  }

  ~VinsKickoff(void) {
  }

  void initialize(ros::NodeHandle &nh, const std::shared_ptr<CommonHandlers_t> &ch,
                  const std::shared_ptr<pairs_uav_managers::estimation_manager::PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  bool setUavState(const pairs_msgs::UavState &uav_state) override;
};

}  // namespace vins_kickoff

#endif  // ESTIMATORS_STATE_vins_kickoff_H
