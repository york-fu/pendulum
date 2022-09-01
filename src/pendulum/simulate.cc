#include "simulate.h"
#include <gflags/gflags.h>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include "utils.h"
#include "lcm_publish.h"
#include "lqr_control.h"

namespace drake {
namespace examples {
namespace pendulum {

using trajectories::PiecewisePolynomial;

uint8_t get_trajectory(Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* tor)
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
  const double time_cost = 1;  //s
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
  const double timespan_init = time_cost;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.value(), final_state.value()});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  // std::cout << "traj_init_x.value(0.51):" << traj_init_x.value(0.51) << std::endl;

  /* 求解 */
  struct timespec solve_time;
  clock_gettime(CLOCK_MONOTONIC, &solve_time);
  double runtime = solve_time.tv_sec * 1e3 + solve_time.tv_nsec * 1e-6;

  const auto result = solvers::Solve(dircol.prog());

  clock_gettime(CLOCK_MONOTONIC, &solve_time);
  runtime = solve_time.tv_sec * 1e3 + solve_time.tv_nsec * 1e-6 - runtime;
  printf("solve_runtime: %f ms\n", runtime);

  Eigen::VectorXd t;
  t.resize(kNumTimeSamples);
  for(int i = 0; i < kNumTimeSamples; i++)
  {
    t(i) = kMinimumTimeStep * i;
  }
  Eigen::MatrixXd status = dircol.GetStateSamples(result);
  *pos = status.row(0);  //rad
  *vel = status.row(1);  //rad/s
  Eigen::MatrixXd input = dircol.GetInputSamples(result);
  *tor = input.row(0);  //N·m
  // auto traj_pos = PiecewisePolynomial<double>::FirstOrderHold(t, pos.transpose());
  // auto traj_vel = PiecewisePolynomial<double>::FirstOrderHold(t, vel.transpose());
  // auto traj_tor = PiecewisePolynomial<double>::FirstOrderHold(t, tor.transpose());
  // std::ofstream file;
  // file.open("/home/chen/Desktop/test.txt", std::ios::out | std::ios::app);
  // for(int i = 0; i < 1000; i++)
  // {
  //   file << "t: " << i << " ms"
  //       << "  pos: " << traj_pos.value(i*0.001)
  //       << "  vel: " << traj_vel.value(i*0.001)
  //       << "  tor: " << traj_tor.value(i*0.001) << std::endl;
  // }
  // file.close();

  /* 打印 */
  // std::cout << "traj_pos.value(1):" << traj_pos.value(1) << std::endl;

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


namespace drake
{
  Simulate::Simulate(multibody::MultibodyPlant<double> *plant) : plant_(plant)
  {
    plant_context_ = plant_->CreateDefaultContext();

    DeclareVectorInputPort("input", 2);  //输入
    DeclareVectorOutputPort("output", 1, &Simulate::OUT);  //输出

    drake::examples::pendulum::get_trajectory(&pos, &vel, &tor);
  }

  void Simulate::OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const
  {
    Eigen::VectorBlock<VectorX<double>> output_vector = output->get_mutable_value();
    output_vector.setZero();

    output_vector << 0;
  }

}; // namespace drake