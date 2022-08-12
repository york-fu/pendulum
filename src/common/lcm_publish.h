#pragma once

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcm_std_msgs/Float64.hpp"
#include "lcm_std_msgs/Float64MultiArray.hpp"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"

void lcmPublishValue(lcm::LCM *lc, std::string name, const double value);
void lcmPublishVector(lcm::LCM *lc, std::string name, const Eigen::VectorXd vec);
void lcmPublishState(lcm::LCM *lc, std::string prefix,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &v, const Eigen::VectorXd &vdot,
                     bool to_rpy = true);
                     