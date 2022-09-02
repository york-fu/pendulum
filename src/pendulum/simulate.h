#ifndef __SIMULATE_H
#define __SIMULATE_H

#include <iostream>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "actuators_interface.h"
#include <lcm/lcm-cpp.hpp>

#include <drake/examples/pendulum/pendulum_plant.h>
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/pendulum/pendulum_geometry.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace examples {
namespace pendulum {

uint8_t get_swingup_trajectory(Eigen::VectorXd &pos, Eigen::VectorXd &vel, Eigen::VectorXd &tor);

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

namespace drake
{
  class Simulate : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Simulate)

    Simulate(multibody::MultibodyPlant<double> *plant, double kp_, double kd_);

  private:
    void OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const;

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    Eigen::VectorXd traj_t;  //s
    Eigen::VectorXd pos;  //rad
    Eigen::VectorXd vel;  //rad/s
    Eigen::VectorXd tor;  //N·m
    int32_t na;  //驱动个数
    int32_t nq;  //位置个数
    int32_t nv;  //速度个数
    mutable double t = 0;
    mutable Eigen::VectorXd q_desire, v_desire;  //目标位置、速度
    mutable Eigen::VectorXd q_measure, v_measure;  //状态位置、速度
    mutable Eigen::VectorXd q_err, v_err;  //误差位置、速度
    mutable Eigen::VectorXd pout, dout;  //pid输出
    mutable Eigen::VectorXd torque_move_fd;  //运动前馈
    mutable Eigen::VectorXd torque_fd;  //状态前馈
    Eigen::VectorXd kp = Eigen::VectorXd::Constant(1, 36.471);
    Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 4.93858);
    mutable Eigen::VectorXd out;  //输出

  };
} // namespace drake

#endif