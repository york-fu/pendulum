#include <memory>
#include <queue>
#include <iostream>
#include <ctime>
#include <signal.h>
#include <gflags/gflags.h>
#include "drake/common/drake_assert.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/adder.h"
#include "make_pendulum_plant.h"
#include "force_disturber.h"
#include "hardware_plant.h"

DEFINE_double(dt, 1.0e-3, "Control period.");
DEFINE_double(realtime, 1.0, "Target realtime rate.");
DEFINE_double(simtime, 60.0, "Simulation time.");
DEFINE_bool(pid, false, "Use PID.");
DEFINE_double(theta, M_PI, "Desired angle.");
DEFINE_bool(pub, true, "Publish lcm msg");
DEFINE_bool(real, false, "Run real");

lcm::LCM lc;

namespace drake
{
  int do_main()
  {
    systems::DiagramBuilder<double> builder;
    systems::lcm::LcmInterfaceSystem *lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
    geometry::SceneGraph<double> *scene_graph = builder.AddSystem<geometry::SceneGraph>();
    pendulum::PendulumParameters parameters(FLAGS_dt, 2, 0.4, 0.2);
    multibody::MultibodyPlant<double> *plant = builder.AddSystem(MakePendulumPlant(parameters, scene_graph));

    builder.Connect(plant->get_geometry_poses_output_port(), scene_graph->get_source_pose_port(plant->get_source_id().value()));
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);

    HardwarePlant *hard_plant;
    if (FLAGS_real)
    {
      hard_plant = builder.AddSystem<HardwarePlant>(plant);
    }

    if (!FLAGS_pid)
    {
      std::cout << "Use LQR.\n";
      auto context = plant->CreateDefaultContext();
      const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(parameters.pin_joint_name());
      joint.set_angle(context.get(), FLAGS_theta);
      Eigen::VectorXd tau(plant->num_actuated_dofs());
      tau.setZero();
      plant->get_actuation_input_port().FixValue(context.get(), tau);
      Eigen::MatrixXd Q(2, 2);
      Q << 100, 0,
          0, 1;
      Eigen::MatrixXd R(1, 1);
      R << 0.1;
      auto N = Eigen::Matrix<double, 0, 0>::Zero();
      auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(*plant, *context.get(),
                                                                                  Q, R, N,
                                                                                  plant->get_actuation_input_port().get_index()));
      std::cout << "A: " << lqr->A() << "\n";
      std::cout << "B: " << lqr->B() << "\n";
      std::cout << "C: " << lqr->C() << "\n";
      std::cout << "D: " << lqr->D() << "\n";
      if (!FLAGS_real)
      {
        builder.Connect(plant->get_state_output_port(), lqr->get_input_port());
        builder.Connect(lqr->get_output_port(), plant->get_actuation_input_port());
      }
      else
      {
        builder.Connect(hard_plant->get_state_output_port(), lqr->get_input_port());
        builder.Connect(lqr->get_output_port(), hard_plant->get_actuation_input_port());
      }
    }
    else
    {
      std::cout << "Use PID.\n";
      Eigen::VectorXd kp = Eigen::VectorXd::Constant(1, 20);
      Eigen::VectorXd ki = Eigen::VectorXd::Constant(1, 0);
      Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 4);
      auto controller = builder.AddSystem<systems::controllers::PidController>(kp, ki, kd);
      auto desired = builder.AddSystem<systems::ConstantVectorSource>(Eigen::Vector2d(FLAGS_theta, 0));
      builder.Connect(desired->get_output_port(), controller->get_input_port_desired_state());
      if (!FLAGS_real)
      {
        builder.Connect(plant->get_state_output_port(), controller->get_input_port_estimated_state());
        builder.Connect(controller->get_output_port_control(), plant->get_actuation_input_port());
      }
      else
      {
        builder.Connect(hard_plant->get_state_output_port(), controller->get_input_port_estimated_state());
        builder.Connect(controller->get_output_port_control(), hard_plant->get_actuation_input_port());
      }
    }

    Eigen::Matrix<double, 6, 1> disturb;
    disturb << 0, 50, 0, 0, 0, 0;
    auto force_disturber = builder.AddSystem<ForceDisturber>(plant->GetBodyByName(parameters.body_name()).index(), disturb, 4, 0.1, 4);
    builder.Connect(force_disturber->get_output_port(), plant->get_applied_spatial_force_input_port());

    auto diagram = builder.Build();
    auto diagram_context = diagram->CreateDefaultContext();

    systems::Context<double> &plant_context = diagram->GetMutableSubsystemContext(*plant, diagram_context.get());
    const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(parameters.pin_joint_name());
    joint.set_angle(&plant_context, FLAGS_theta);
    Eigen::VectorXd qv = plant->GetPositionsAndVelocities(plant_context);
    Eigen::VectorXd q0 = qv.segment(0, plant->num_positions());
    Eigen::VectorXd v0 = qv.segment(plant->num_positions(), plant->num_velocities());

    if (FLAGS_real)
    {
      Eigen::VectorXd tau(plant->num_actuated_dofs());
      tau.setZero();
      plant->get_actuation_input_port().FixValue(&plant_context, tau);

      std::vector<double> q_initial_vec;
      q_initial_vec.assign(&q0[0], q0.data() + q0.rows() * q0.cols());
      for (uint32_t i = 0; i < q_initial_vec.size(); i++)
      {
        q_initial_vec[i] *= (180.0 / M_PI);
      }
      jointMoveTo(q_initial_vec, 90, FLAGS_dt);

      systems::Context<double> &h_plant_context = diagram->GetMutableSubsystemContext(*hard_plant, diagram_context.get());
      hard_plant->Initial(h_plant_context, qv);
      FLAGS_realtime = 1.0;
    }

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.Initialize();
    simulator.set_target_realtime_rate(FLAGS_realtime);
    simulator.AdvanceTo(FLAGS_simtime);

    return 0;
  }

} // namespace drake

void sigintHandler(int sig)
{
  printf("\n");
  HardwarePlantDeInit();
  printf("signal exit.\n");
  exit(EXIT_SUCCESS);
}

int main(int argc, char *argv[])
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!lc.good())
  {
    std::cout << "Error: Lcm not good!\n";
    return -1;
  }
  if (FLAGS_real)
  {
    signal(SIGINT, sigintHandler);
    if (HardwarePlantInit(lc) != 0)
    {
      return -2;
    }
  }
  return drake::do_main();
}
