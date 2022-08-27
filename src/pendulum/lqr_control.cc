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

DEFINE_double(dt, 0.5e-3, "Control period.");
DEFINE_double(realtime, 1.0, "Target realtime rate.");
DEFINE_double(simtime, 60, "Simulation time.");  //仿真时长
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

      // static int32_t t = 0;
      // static int32_t t2 = 0;
      // int32_t T = 10000;

      // if(t2 == 0)
      // {
      //   std::ofstream file;
      //   file.open("/home/chen/Desktop/test.txt", std::ios::out | std::ios::app);
      //   std::cout.setf(std::ios::fixed);
      //   file << "lqr;  Q: 1000, 1000;  state: position, velocities" << std::endl;

      //   file.close();
      // }

      // t++;
      // t2++;
      // t %= T;

      // printf("%d\r",t2);
      // if(t2 <= 5000)
      // {
      //     std::ofstream file;
      //     file.open("/home/chen/Desktop/test.txt", std::ios::out | std::ios::app);
      //     std::cout.setf(std::ios::fixed);
      //     file << std::setfill(' ') << std::setw(4) << t2 << "   " 
      //          << std::fixed << std::setprecision(7) << q_(0) << "   " 
      //          << std::fixed << std::setprecision(7) << v_(0) << ""
      //          << std::endl;
      //     file.close();
      // }
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
    
    geometry::SceneGraph<double> *scene_graph = builder.AddSystem<geometry::SceneGraph>();  //创建scene_graph
    pendulum::PendulumParameters parameters(FLAGS_dt, FLAGS_mass, FLAGS_length, FLAGS_damping);  //dt = 1e-3, mass = 1.0, length = 0.5, damping = 0.1, gravity = 9.81
    multibody::MultibodyPlant<double> *plant = builder.AddSystem(MakePendulumPlant(parameters, scene_graph));  //创建plant
    builder.Connect(plant->get_geometry_poses_output_port(), scene_graph->get_source_pose_port(plant->get_source_id().value()));  //连接plant和对应scene_graph

    systems::lcm::LcmInterfaceSystem *lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();  //创建lcm
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, lcm);  //

    auto state_sender = builder.AddSystem<StateSender>(plant);  //创建state_sender

    HardwarePlant *hard_plant;
    if (FLAGS_real)
    {
      hard_plant = builder.AddSystem<HardwarePlant>(plant);  //创建hard_plant
    }

    if (!FLAGS_pid)
    {
      std::cout << "Use LQR.\n";

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
      auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(*plant, *context.get(),
                                                                                  Q, R, N,
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

      if (!FLAGS_real)
      {
        builder.Connect(plant->get_state_output_port(), lqr->get_input_port());
        builder.Connect(lqr->get_output_port(), plant->get_actuation_input_port());
        builder.Connect(plant->get_state_output_port(), state_sender->get_state_input_port());
      }
      else
      {
        builder.Connect(hard_plant->get_state_output_port(), lqr->get_input_port());
        builder.Connect(lqr->get_output_port(), hard_plant->get_actuation_input_port());
        builder.Connect(hard_plant->get_state_output_port(), state_sender->get_state_input_port());
      }
    }
    else
    {
      std::cout << "Use PID.\n";
      // Eigen::VectorXd kp = Eigen::VectorXd::Constant(1, 10);
      // Eigen::VectorXd ki = Eigen::VectorXd::Constant(1, 0);
      // Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 4);
      // auto controller = builder.AddSystem<systems::controllers::PidController>(kp, ki, kd);
      // auto desired = builder.AddSystem<systems::ConstantVectorSource>(Eigen::Vector2d(FLAGS_theta, 0));
      // builder.Connect(desired->get_output_port(), controller->get_input_port_desired_state());

      auto feedfor = builder.AddSystem<drake::lqrfeedforward>(plant, parameters);  //创建pid+前馈
      auto command_sender = builder.AddSystem<CommandSender>(plant);  //创建command_sender

      if (!FLAGS_real)
      {
        // builder.Connect(plant->get_state_output_port(), controller->get_input_port_estimated_state());
        // builder.Connect(controller->get_output_port_control(), plant->get_actuation_input_port());
        // builder.Connect(plant->get_state_output_port(), state_sender->get_state_input_port());

        builder.Connect(plant->get_state_output_port(), feedfor->get_input_port());
        builder.Connect(feedfor->get_output_port(0), plant->get_actuation_input_port());
        builder.Connect(plant->get_state_output_port(), state_sender->get_state_input_port());
        builder.Connect(feedfor->get_output_port(1), command_sender->get_desired_input_port());
      }
      else
      {
        // builder.Connect(hard_plant->get_state_output_port(), controller->get_input_port_estimated_state());
        // builder.Connect(controller->get_output_port_control(), hard_plant->get_actuation_input_port());
        // builder.Connect(hard_plant->get_state_output_port(), state_sender->get_state_input_port());

        builder.Connect(hard_plant->get_state_output_port(), feedfor->get_input_port());
        builder.Connect(feedfor->get_output_port(0), hard_plant->get_actuation_input_port());
        builder.Connect(hard_plant->get_state_output_port(), state_sender->get_state_input_port());
        builder.Connect(feedfor->get_output_port(1), command_sender->get_desired_input_port());
      }
    }

    Eigen::Matrix<double, 6, 1> disturb;  //扰动
    disturb << 0, 50, 0, 0, 0, 0;
    auto force_disturber = builder.AddSystem<ForceDisturber>(plant->GetBodyByName(parameters.body_name()).index(), disturb, 0, 0, 4);  //创建力扰动
    builder.Connect(force_disturber->get_output_port(), plant->get_applied_spatial_force_input_port());  //连接力扰动

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
