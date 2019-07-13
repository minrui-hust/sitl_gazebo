// Copyright 2019 Minrui. All Rights Reserved.
// Author: Minrui (hustminrui@126.com).

#include "gazebo_datalogger_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboDataLoggerPlugin::GazeboDataLoggerPlugin() : ModelPlugin() {}

GazeboDataLoggerPlugin::~GazeboDataLoggerPlugin() {
  fout_.close();
  updateConnection_->~Connection();
}

void GazeboDataLoggerPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_datalogger_plugin] Please specify a linkName.\n";

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_datalogger_plugin] Couldn't find specified link \""
            << link_name_ << "\".");

  std::string log_path;
  getSdfParam<std::string>(_sdf, "logPath", log_path, "/home/mr/");

  // struct timeval s_now;
  // struct tm *p_tm;
  // gettimeofday(&s_now, NULL);
  // p_tm = localtime((const time_t *)&s_now.tv_sec);
  // p_tm->tm_year += 1900;
  // std::string log_file_name =
  //    log_path + "/datalogger_" + std::to_string(p_tm->tm_year) + "-" +
  //    std::to_string(p_tm->tm_mon) + "-" + std::to_string(p_tm->tm_mday) + "-"
  //    + std::to_string(p_tm->tm_hour) + "-" + std::to_string(p_tm->tm_min) +
  //    "-" + std::to_string(p_tm->tm_sec) + ".log";
  std::string log_file_name = log_path + "/datalogger.log";
  fout_.open(log_file_name, std::ios::trunc);
  fout_ << "time,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,ax,ay,az,wwx,wwy,wwz"
        << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboDataLoggerPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboDataLoggerPlugin::OnUpdate(const common::UpdateInfo &_info) {
  // get pose
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose(); // TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I =
      ignitionFromGazeboMath(link_->GetWorldPose()); // TODO(burrimi): Check tf.
#endif

  // get vel in body frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d angular_vel_I = link_->RelativeAngularVel();
  ignition::math::Vector3d linear_vel_I = link_->RelativeLinearVel();
#else
  ignition::math::Vector3d angular_vel_I =
      ignitionFromGazeboMath(link_->GetRelativeAngularVel());
  ignition::math::Vector3d linear_vel_I =
      ignitionFromGazeboMath(link_->GetRelativeLinearVel());
#endif

  // get acc in body frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d linear_accel_I = link_->RelativeLinearAccel();
  ignition::math::Vector3d angular_accel_I = link_->RelativeAngularAccel();
#else
  ignition::math::Vector3d linear_accel_I =
      ignitionFromGazeboMath(link_->GetRelativeLinearAccel());
  ignition::math::Vector3d angular_accel_I =
      ignitionFromGazeboMath(link_->GetRelativeAngularAccel());
#endif

  // log the full state
  // clang-format off
  fout_ << std::setprecision(20)  <<

    _info.simTime.Double() << "," << 

    T_W_I.Pos().X()     << "," << 
    T_W_I.Pos().Y()     << "," << 
    T_W_I.Pos().Z()     << "," << 

    T_W_I.Rot().W()     << "," <<
    T_W_I.Rot().X()     << "," <<
    T_W_I.Rot().Y()     << "," <<
    T_W_I.Rot().Z()     << "," << 

    linear_vel_I.X()    << "," << 
    linear_vel_I.Y()    << "," << 
    linear_vel_I.Z()    << "," << 

    angular_vel_I.X()   << "," <<
    angular_vel_I.Y()   << "," <<
    angular_vel_I.Z()   << "," <<

    linear_accel_I.X()  << "," << 
    linear_accel_I.Y()  << "," << 
    linear_accel_I.Z()  << "," << 

    angular_accel_I.X() << "," << 
    angular_accel_I.Y() << "," << 
    angular_accel_I.Z() << "," << 
    std::endl;
  // clang-format on
}

GZ_REGISTER_MODEL_PLUGIN(GazeboDataLoggerPlugin);
} // namespace gazebo
