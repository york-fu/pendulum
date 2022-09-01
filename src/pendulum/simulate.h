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

uint8_t get_trajectory(Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* tor);

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

namespace drake
{
  class Simulate : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Simulate)

    Simulate(multibody::MultibodyPlant<double> *plant);

  private:
    void OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const;

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    Eigen::VectorXd pos;  //rad
    Eigen::VectorXd vel;  //rad/s
    Eigen::VectorXd tor;  //N·m
  };
} // namespace drake

#endif