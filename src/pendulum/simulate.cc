#include "simulate.h"
#include <gflags/gflags.h>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include "utils.h"
#include "lcm_publish.h"
#include "lqr_control.h"
#include <fstream>

namespace drake {
namespace examples {
namespace pendulum {

using trajectories::PiecewisePolynomial;

uint8_t get_swingup_trajectory(Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* tor)
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
  const double traj_time = FLAGS_traj_time;  //s
  const int kNumTimeSamples = FLAGS_traj_point_num;
  const double kMinimumTimeStep = traj_time / kNumTimeSamples;
  const double kMaximumTimeStep = traj_time / kNumTimeSamples;
  systems::trajectory_optimization::DirectCollocation dircol(
      pendulum.get(), *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumTimeStep);
  auto& prog = dircol.prog();

  dircol.AddEqualTimeIntervalsConstraints();

  /* 控制力矩约束 */
  const double kTorqueLimit = FLAGS_traj_torque_limit;  // N*m.
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
  const double timespan_init = traj_time;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.value(), final_state.value()});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);

  /* 求解 */
  struct timespec solve_time;
  clock_gettime(CLOCK_MONOTONIC, &solve_time);
  double runtime = solve_time.tv_sec * 1e3 + solve_time.tv_nsec * 1e-6;

  const auto result = solvers::Solve(dircol.prog());

  clock_gettime(CLOCK_MONOTONIC, &solve_time);
  runtime = solve_time.tv_sec * 1e3 + solve_time.tv_nsec * 1e-6 - runtime;
  printf("solve_runtime: %f ms\n", runtime);

  /* 判断是否成功 */
  if (!result.is_success()) {
    std::cerr << "Failed to solve optimization for the swing-up trajectory"
              << std::endl;
    pos->setZero(kNumTimeSamples, 1);
    vel->setZero(kNumTimeSamples, 1);
    tor->setZero(kNumTimeSamples, 1);
    return 1;
  }

  Eigen::MatrixXd status = dircol.GetStateSamples(result);
  *pos = status.row(0);  //rad  列向量
  *vel = status.row(1);  //rad/s
  Eigen::MatrixXd input = dircol.GetInputSamples(result);
  *tor = input.row(0);  //N·m

  return 0;
}

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

namespace drake
{
  Simulate::Simulate(multibody::MultibodyPlant<double> *plant, double kp_, double kd_) : plant_(plant)
  {
    plant_context_ = plant_->CreateDefaultContext();

    na = plant_->num_actuated_dofs();
    nq = plant_->num_positions();
    nv = plant_->num_velocities();

    kp << kp_;
    kd << kd_;

    DeclareVectorInputPort("input", nq+nv);  //输入
    DeclareVectorOutputPort("output", na, &Simulate::OUT);  //输出

    q_desire.resize(nq);
    v_desire.resize(nv);
    q_measure.resize(nq);
    v_measure.resize(nv);
    q_err.resize(nq);
    v_err.resize(nv);
    pout.resize(nq);
    dout.resize(nv);
    torque_move_fd.resize(pout.size());
    torque_fd.resize(pout.size());
    out.resize(pout.size());
    out.setZero();

    drake::examples::pendulum::get_swingup_trajectory(&pos, &vel, &tor);

    traj_t.resize(FLAGS_traj_point_num);
    for(int i = 0; i < FLAGS_traj_point_num; i++)
    {
      traj_t(i) = FLAGS_traj_time / FLAGS_traj_point_num * i;
    }
  }

  void Simulate::OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const
  {
    Eigen::VectorBlock<VectorX<double>> output_vector = output->get_mutable_value();
    output_vector.setZero();

    /* 常量 */
    const double g = 9.81; // gravity
    const double rad2deg = 180./M_PI;

    /* 时间戳 */
    t += FLAGS_dt;

    /* 轨迹 */
    static auto traj_pos = examples::pendulum::PiecewisePolynomial<double>::FirstOrderHold(traj_t, pos.transpose());
    static auto traj_vel = examples::pendulum::PiecewisePolynomial<double>::FirstOrderHold(traj_t, vel.transpose());
    static auto traj_tor = examples::pendulum::PiecewisePolynomial<double>::FirstOrderHold(traj_t, tor.transpose());

    /* 目标值 */
    q_desire = traj_pos.value(t);
    v_desire = traj_vel.value(t);
    if(t > FLAGS_traj_time)
    {
      q_desire << FLAGS_theta;
      v_desire << 0;
    }

    /* 状态值 */
    const auto qv = get_input_port().Eval(context);
    q_measure = qv.segment(0, nq);  //位置（仿真是rad）
    v_measure = qv.segment(nq, nv);  //速度（仿真是rad/s）

    /* 误差值 */
    q_err = q_desire - q_measure;
    v_err = v_desire - v_measure;

    /* 位置速度pp控制 */
    pout = kp * q_err;
    dout = kd * v_err;
    
    /* 运动前馈 */
    torque_move_fd = traj_tor.value(t);
    if(t > FLAGS_traj_time)
    {
      torque_move_fd << 0;
    }

    /* 状态前馈 */
    torque_fd << (FLAGS_mass*FLAGS_length)*g*sin(q_measure[0]);

    /* 输出 */
    out = torque_move_fd + pout + dout;// + torque_fd;
    if(out(0) > 44.12)
    {
      output_vector << 44.12;
    }
    else if(out(0) < -44.12)
    {
      output_vector << -44.12;
    }
    else 
    {
      output_vector << out;
    }

    /* 打印 */
  //   std::ofstream file;
  //   file.open("/home/chen/Desktop/test.txt", std::ios::out | std::ios::app);
  //   file << t
  //       << "  " << q_measure(0)
  //       << "  " << v_measure(0)
  //       << "  " << output_vector(0)
  //       << "  " << torque_move_fd(0) 
  //       << "  " << pout + dout << std::endl;
  //       // << "  " << traj_tor.value(t)
  //   file.close();
  }

}; // namespace drake