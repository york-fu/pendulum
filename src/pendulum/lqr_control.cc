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
#include "lcm_publish.h"
#include "lqrfeedforward.h"
#include <fstream>
#include "iomanip"

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

uint8_t get_trajectory(void);

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

DEFINE_double(dt, 0.5e-3, "Control period.");
DEFINE_double(realtime, 1.0, "Target realtime rate.");
DEFINE_double(simtime, 2, "Simulation time.");  //仿真时长
DEFINE_bool(pid, false, "Use PID.");
DEFINE_double(theta, M_PI, "Desired angle.");
DEFINE_bool(pub, true, "Publish lcm msg");
DEFINE_bool(real, false, "Run real");
DEFINE_double(mass, 1.21, "Parameter mass");
DEFINE_double(length, 0.41, "Parameter length");
DEFINE_double(damping, 0, "Parameter damping");
DEFINE_double(init_pos, 0, "init position");  //弧度（仿真和实物都会初始化位置）
DEFINE_double(Q1, 100, "Q's parameter1");
DEFINE_double(Q2, 1, "Q's parameter2");

lcm::LCM lc;

namespace drake
{
  class StateSender : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateSender)

    StateSender(multibody::MultibodyPlant<double> *plant) : plant_(plant)
    {
      plant_context_ = plant_->CreateDefaultContext();
      na_ = plant_->num_actuated_dofs();
      nq_ = plant_->num_positions();
      nv_ = plant_->num_velocities();
      dt_ = plant_->time_step();
      DeclareVectorInputPort("state", nq_ + nv_);
      DeclarePeriodicDiscreteUpdateEvent(dt_, 0, &StateSender::Update);
      q_.resize(nq_);
      v_.resize(nv_);
      vd_.resize(nv_);
      prev_v_.resize(nv_);
      q_.setZero();
      v_.setZero();
    }

    const systems::InputPort<double> &get_state_input_port() const
    {
      return systems::LeafSystem<double>::get_input_port(0);
    }

  private:
    void Update(const systems::Context<double> &context, systems::DiscreteValues<double> *next_state) const
    {
      const auto &qv = get_state_input_port().Eval(context);
      prev_v_ = v_;
      q_ = qv.segment(0, nq_);
      v_ = qv.segment(nq_, nv_);
      vd_ = (v_ - prev_v_) / dt_;

      if (FLAGS_pub)
      {
        lcmPublishState(&lc, "state", q_, v_, vd_, false);
      }
    }

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    double dt_;
    mutable Eigen::VectorXd q_, v_, vd_;
    mutable Eigen::VectorXd prev_v_;
  };
} // namespace drake

namespace drake
{
  class CommandSender : public systems::LeafSystem<double>
  {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CommandSender)

    CommandSender(multibody::MultibodyPlant<double> *plant) : plant_(plant)
    {
      plant_context_ = plant_->CreateDefaultContext();
      na_ = plant_->num_actuated_dofs();
      nq_ = plant_->num_positions();
      nv_ = plant_->num_velocities();
      dt_ = plant_->time_step();
      DeclareVectorInputPort("command", nq_ + nv_);
      DeclarePeriodicDiscreteUpdateEvent(dt_, 0, &CommandSender::Update);
      q_.resize(nq_);
      v_.resize(nv_);
      vd_.resize(nv_);
      prev_v_.resize(nv_);
      q_.setZero();
      v_.setZero();
    }

    const systems::InputPort<double> &get_desired_input_port() const
    {
      return systems::LeafSystem<double>::get_input_port(0);
    }

  private:
    void Update(const systems::Context<double> &context, systems::DiscreteValues<double> *next_state) const
    {
      const auto &qv_des = get_desired_input_port().Eval(context);
      prev_v_ = v_;
      q_ = qv_des.segment(0, nq_);
      v_ = qv_des.segment(nq_, nv_);
      vd_ = (v_ - prev_v_) / dt_;

      if (FLAGS_pub)
      {
        lcmPublishState(&lc, "desire", q_, v_, vd_, false);
      }
    }

    multibody::MultibodyPlant<double> *plant_;
    std::unique_ptr<systems::Context<double>> plant_context_;
    int32_t na_;
    int32_t nq_;
    int32_t nv_;
    double dt_;
    mutable Eigen::VectorXd q_, v_, vd_;
    mutable Eigen::VectorXd prev_v_;
  };
} // namespace drake

namespace drake
{
  int do_main()
  {
    systems::DiagramBuilder<double> builder;
    
    /* 创建 */
    geometry::SceneGraph<double> *scene_graph = builder.AddSystem<geometry::SceneGraph>();  //创建scene_graph
    pendulum::PendulumParameters parameters(FLAGS_dt, FLAGS_mass, FLAGS_length, FLAGS_damping);  //创建parameters
    multibody::MultibodyPlant<double> *plant = builder.AddSystem(MakePendulumPlant(parameters, scene_graph));  //创建plant
    systems::lcm::LcmInterfaceSystem *lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();  //创建lcm
    HardwarePlant *hard_plant;  //创建hard_plant

    /* 匹配 */
    builder.Connect(plant->get_geometry_poses_output_port(), scene_graph->get_source_pose_port(plant->get_source_id().value()));  //连接plant和对应scene_graph
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);  //

    /* 仿真 */
    if (!FLAGS_real)
    {
      std::cout << "\nUse LQR.\n";

      /* lqr */
      auto context = plant->CreateDefaultContext();
      const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(parameters.pin_joint_name());  //创建关节
      joint.set_angle(context.get(), FLAGS_theta);  //设置关节目标角度
      Eigen::VectorXd tau(plant->num_actuated_dofs());
      tau.setZero();
      plant->get_actuation_input_port().FixValue(context.get(), tau);
      Eigen::MatrixXd Q(2, 2);
      Q << FLAGS_Q1, 0,
          0, FLAGS_Q2;
      Eigen::MatrixXd R(1, 1);
      R << 0.1;
      auto N = Eigen::Matrix<double, 0, 0>::Zero();
      auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(*plant, *context.get(), Q, R, N,
                                                                                  plant->get_actuation_input_port().get_index()));  //创建lqr
      std::cout << "D: " << lqr->D() << "\n";
      auto PD = lqr->D();
      PD(0) /= PD(1);
      PD(1) *= 3.14159 / 180.0 / 3276.8;
      std::cout << "P: " << PD(0) << "  D: " << PD(1) << std::endl;
      PD = lqr->D();
      PD(0) *= 3.14159 / 180.0;
      PD(1) *= 3.14159 / 180.0;
      std::cout << "P: " << PD(0) << "  D: " << PD(1) << std::endl;

      /* 连接 */
      builder.Connect(plant->get_state_output_port(), lqr->get_input_port());
      builder.Connect(lqr->get_output_port(), plant->get_actuation_input_port());

      /* state_sender */
      // auto state_sender = builder.AddSystem<StateSender>(plant);  //创建state_sender
      // builder.Connect(plant->get_state_output_port(), state_sender->get_state_input_port());

      /* 扰动 */
      Eigen::Matrix<double, 6, 1> disturb;
      disturb << 0, 50, 0, 0, 0, 0;
      auto force_disturber = builder.AddSystem<ForceDisturber>(plant->GetBodyByName(parameters.body_name()).index(), disturb, 0, 0, 4);  //创建力扰动
      builder.Connect(force_disturber->get_output_port(), plant->get_applied_spatial_force_input_port());  //连接力扰动

      drake::examples::pendulum::get_trajectory();
    }
    /* 实物 */
    else
    {
      std::cout << "\nReal.\n";

      /* 创建 */
      hard_plant = builder.AddSystem<HardwarePlant>(plant);  //创建hard_plant;
      auto state_sender = builder.AddSystem<StateSender>(plant);  //创建state_sender
      auto command_sender = builder.AddSystem<CommandSender>(plant);  //创建command_sender
    
      /* 连接 */
      builder.Connect(hard_plant->get_state_output_port(), state_sender->get_state_input_port());
      builder.Connect(hard_plant->get_command_output_port(), command_sender->get_desired_input_port());

      /* 轨迹优化 */
      // drake::examples::pendulum::get_trajectory();
    }

    auto diagram = builder.Build();
    auto diagram_context = diagram->CreateDefaultContext();

    systems::Context<double> &plant_context = diagram->GetMutableSubsystemContext(*plant, diagram_context.get());
    const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(parameters.pin_joint_name());
    joint.set_angle(&plant_context, FLAGS_init_pos);  //设置初始角度（弧度制）
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
      jointMoveTo(q_initial_vec, 60, FLAGS_dt);

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

  if (!lc.good())  //lcm相关
  {
    std::cout << "Error: Lcm not good!\n";
    return -1;
  }

  if (FLAGS_real)  //实物相关
  {
    signal(SIGINT, sigintHandler);
    if (HardwarePlantInit(lc) != 0)
    {
      return -2;
    }
  }

  return drake::do_main();
}

namespace drake {
namespace examples {
namespace pendulum {

using trajectories::PiecewisePolynomial;

uint8_t get_trajectory(void)
{
  auto pendulum = std::make_unique<PendulumPlant<double>>();
  pendulum->set_name("pendulum");
  auto context = pendulum->CreateDefaultContext();

  /* 模型 */
  auto& parameters = pendulum->get_mutable_parameters(context.get());
  parameters.set_mass(FLAGS_mass);
  parameters.set_length(FLAGS_length);
  parameters.set_damping(FLAGS_damping);

  /* 轨迹配置 */
  const double time_cost = 0.5;  //s
  const double frequency = 100;  //Hz
  const int kNumTimeSamples = (int)(time_cost * frequency);
  const double kMinimumTimeStep = 1.0 / frequency;//0.01;
  const double kMaximumTimeStep = 1.0 / frequency;//0.01;
  systems::trajectory_optimization::DirectCollocation dircol(
      pendulum.get(), *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep);
  auto& prog = dircol.prog();

  dircol.AddEqualTimeIntervalsConstraints();

  /* 控制力矩约束 */
  const double kTorqueLimit = 20;  // N*m.
  const solvers::VectorXDecisionVariable& u = dircol.input();
  dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  /* 初末状态 */
  PendulumState<double> initial_state, final_state;
  initial_state.set_theta(M_PI_2);
  initial_state.set_thetadot(0.0);
  final_state.set_theta(M_PI);
  final_state.set_thetadot(0.0);

  prog.AddLinearConstraint(dircol.initial_state() == initial_state.value());
  prog.AddLinearConstraint(dircol.final_state() == final_state.value());

  /* 代偿 */
  const double R = 10;  // Cost on input "effort".
  dircol.AddRunningCost((R * u) * u);

  /* 设置初始轨迹 */
  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.value(), final_state.value()});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);

  // std::cout << "decision_variable: " << prog.decision_variables() << std::endl;

  /* 求解 */
  struct timespec solve_time;
  clock_gettime(CLOCK_MONOTONIC, &solve_time);
  double runtime = solve_time.tv_sec * 1e3 + solve_time.tv_nsec * 1e-6;

  const auto result = solvers::Solve(dircol.prog());

  clock_gettime(CLOCK_MONOTONIC, &solve_time);
  runtime = solve_time.tv_sec * 1e3 + solve_time.tv_nsec * 1e-6 - runtime;
  printf("solve_runtime: %f ms\n", runtime);

  Eigen::MatrixXd status = dircol.GetStateSamples(result);
  Eigen::VectorXd pos = status.row(0);  //rad
  Eigen::VectorXd vel = status.row(1);  //rad/s
  Eigen::MatrixXd output = dircol.GetInputSamples(result);  //N·m
  // std::ofstream file;
  // file.open("/home/chen/Desktop/test.txt", std::ios::out | std::ios::app);
  // file << result.GetSolution() << std::endl;
  // file.close();

  /* 判断是否成功 */
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the swing-up trajectory"
              << std::endl;
    return 1;
  }

  return 0;
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
