// Copyright 2019 Minrui. All Rights Reserved.
// Author: Minrui (hustminrui@126.com).

#include <random>

#include "Imu.pb.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <Eigen/Core>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include "common.h"

namespace gazebo {

class GazeboDataLoggerPlugin : public ModelPlugin {
public:
  GazeboDataLoggerPlugin();
  ~GazeboDataLoggerPlugin();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void OnUpdate(const common::UpdateInfo &);

private:
  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  std::string link_name_;

  ignition::math::Vector3d gravity_W_;

  std::ofstream fout_;
};
} // namespace gazebo
