#pragma once

#include <iostream>
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "actuators_interface.h"
#include <lcm/lcm-cpp.hpp>

int8_t HardwarePlantInit(lcm::LCM &lcm_obj);
void HardwarePlantDeInit();
void jointMoveTo(std::vector<double> &goal_pos, double speed, double dt);
void csp_test();

namespace drake
{
  class HardwarePlant : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HardwarePlant)

    HardwarePlant(multibody::MultibodyPlant<double> *plant);

    const systems::OutputPort<double> &get_command_output_port() const
    {
      return systems::LeafSystem<double>::get_output_port(0);
    }

    const systems::OutputPort<double> &get_state_output_port() const
    {
      return systems::LeafSystem<double>::get_output_port(1);
    }

    void Initial(systems::Context<double> &context, Eigen::VectorXd qv) const;

  private:
    void Update(const systems::Context<double> &context, systems::DiscreteValues<double> *next_state) const;
    void CommandOutput(const systems::Context<double> &context, systems::BasicVector<double> *output) const;
    void StateOutput(const systems::Context<double> &context, systems::BasicVector<double> *output) const;

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    uint32_t nq_f_;
    uint32_t nv_f_;
    double dt_;
    const systems::CacheEntry *actuation_cache_{};
  };
} // namespace drake