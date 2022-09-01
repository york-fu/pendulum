#include "hardware_plant.h"
#include <gflags/gflags.h>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include "utils.h"
#include "lcm_publish.h"


DECLARE_bool(pub);
DEFINE_double(dt_, 1e-3, "dt");

#define NUM_JOINT_MAX 12
#define BIT_17 (1 << 17)
#define BIT_17_9 (BIT_17 * 9)
#define C2T_COEFFICIENT (1.44)
#define MAX_TORQUE (44.14)
#define MAX_CURRENT (MAX_TORQUE / C2T_COEFFICIENT)

const char *ifname = "eno1";
uint32_t encoder_range[] = {BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9,
                            BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9, BIT_17_9};

static ActuatorsInterface_t actuators;

static uint32_t num_joint = 1;
static double_t min_joint_position_limits[NUM_JOINT_MAX] = {-90};
static double_t max_joint_position_limits[NUM_JOINT_MAX] = {360};
static double_t joint_offset[NUM_JOINT_MAX] = {0};
static uint8_t joint_ids[NUM_JOINT_MAX] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static JointParam_t joint_data[NUM_JOINT_MAX];
static JointParam_t joint_data_old[NUM_JOINT_MAX];
static JointParam_t joint_cmd[NUM_JOINT_MAX];
static JointParam_t joint_cmd_old[NUM_JOINT_MAX];

static lcm::LCM *lcm_ptr;

static double calcCos(double start, double stop, double T, double t)
{
  double A = (stop - start) / 2.0;
  return A * -cos(M_PI / T * t) + start + A;
}

void jointMoveTo(std::vector<double> &goal_pos, double speed, double dt)
{
  actuators.getJointData(joint_ids, num_joint, joint_data);
  std::vector<double> start_pos(num_joint, 0);
  std::vector<double> T(num_joint, 0);
  for (uint32 i = 0; i < num_joint; i++)
  {
    start_pos[i] = joint_data[i].position;
    T[i] = fabs(goal_pos[i] - start_pos[i]) / speed;
    printf("Joint %d from %f to %f\n", i + 1, start_pos[i], goal_pos[i]);
  }
  double max_T = *max_element(T.begin(), T.end());
  if (max_T < 0.5)
  {
    max_T = 0.5;
  }
  printf("Duration %f\n", max_T);
  uint32_t total_cnt = ceil(max_T / dt);
  uint32_t count = 0;
  struct timespec next_time;
  clock_gettime(CLOCK_MONOTONIC, &next_time);
  while (1)
  {
    for (uint32 i = 0; i < num_joint; i++)
    {
      joint_cmd[i].position = calcCos(start_pos[i], goal_pos[i], total_cnt, count);
      joint_cmd[i].maxTorque = MAX_CURRENT;
    }
    actuators.setJointPosition(joint_ids, num_joint, joint_cmd);

    count++;
    if (count > total_cnt)
    {
      osal_usleep(1000 * 4);
      actuators.getJointData(joint_ids, num_joint, joint_data);
      break;
    }

    next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
    next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }
}

static void checkJointPos(JointParam_t *joint_data)
{
  for (uint32_t i = 0; i < num_joint; i++)
  {
    if (joint_data[i].position < min_joint_position_limits[i] ? true : false)
    {
      printf("Error: Joint %d position minimum value exceeded, actual value %f\n", i + 1, joint_data[i].position);
      exit(EXIT_FAILURE);
    }
    if (joint_data[i].position > max_joint_position_limits[i] ? true : false)
    {
      printf("Error: Joint %d position maximum value exceeded, actual value %f\n", i + 1, joint_data[i].position);
      exit(EXIT_FAILURE);
    }
  }
}

static void readJoint(uint32_t na, double dt,
                      Eigen::VectorXd &joint_q,
                      Eigen::VectorXd &joint_v,
                      Eigen::VectorXd &joint_vd,
                      Eigen::VectorXd &joint_tau)
{
  for (uint32_t i = 0; i < na; i++)
  {
    joint_data_old[i] = joint_data[i];
  }
  actuators.getJointData(joint_ids, na, joint_data);
  checkJointPos(joint_data);
  for (uint32_t i = 0; i < na; i++)
  {
    joint_data[i].acceleration = (joint_data[i].velocity - joint_data_old[i].velocity) / dt;
    joint_q[i] = joint_data[i].position * (M_PI / 180.0);
    joint_v[i] = joint_data[i].velocity * (M_PI / 180.0);
    joint_vd[i] = joint_data[i].acceleration * (M_PI / 180.0);
    joint_tau[i] = joint_data[i].torque * C2T_COEFFICIENT;
  }
}

static void writeJoint(uint32_t na, const Eigen::VectorXd &cmd)
{
  for (uint32_t i = 0; i < na; i++)
  {
    joint_cmd_old[i] = joint_cmd[i];
  }
  for (uint32_t i = 0; i < na; i++)
  {
    joint_cmd[i].torqueOffset = 0;
    joint_cmd[i].torque = cmd[i] / C2T_COEFFICIENT;
    joint_cmd[i].maxTorque = MAX_CURRENT;
  }
  actuators.setJointTorque(joint_ids, na, joint_cmd);
}

int8_t HardwarePlantInit(lcm::LCM &lcm_obj)
{
  lcm_ptr = &lcm_obj;

  actuatorsInterfaceSetup("real", &actuators);
  MotorOptions_t motor_opt;
  motor_opt.size = NUM_JOINT_MAX;
  motor_opt.encoder_range = encoder_range;
  motor_opt.position_limit = 0;
  if (actuators.init(ifname, 1.0e-3, motor_opt) != 0)
  {
    return -1;
  }
#if 0
    exit(0);
#endif
  std::vector<std::vector<double_t>> offset;
  std::string offset_file(__FILE__);
  offset_file.erase(offset_file.rfind("/"));
  offset_file += "/config/offset.csv";
  if (!readCsvData(offset_file.c_str(), false, offset))
  {
    exit(0);
  }
  uint32_t num_onelen_item = 1;
  uint32_t i, j;
  for (i = 0; i < offset.size(); i++)
  {
    for (j = 0; j < num_onelen_item; j++)
    {
      if (i * num_onelen_item + j < NUM_JOINT_MAX)
      {
        joint_offset[i * num_onelen_item + j] = offset[i][j];
      }
    }
  }
  actuators.setJointOffset(joint_offset, NUM_JOINT_MAX);

  // std::vector<double> inital_pos(NUM_JOINT_MAX, 0);
  // jointMoveTo(inital_pos, 60, 1.0e-3);
  return 0;
}

void HardwarePlantDeInit()
{
  std::vector<double> goal_pos(NUM_JOINT_MAX, 0);
  jointMoveTo(goal_pos, 60, 1.0e-3);
  // Eigen::VectorXd cmd(1);
  // cmd << 0;
  // writeJoint(1,cmd);

  usleep(1000 * 10);
  actuators.deInit();
}

namespace drake
{
  HardwarePlant::HardwarePlant(multibody::MultibodyPlant<double> *plant) : plant_(plant)
  {
    plant_context_ = plant_->CreateDefaultContext();
    na_ = plant_->num_actuated_dofs();
    nq_ = plant_->num_positions();
    nv_ = plant_->num_velocities();
    dt_ = FLAGS_dt_;
    Eigen::VectorXd initial_state(nq_ + nv_);
    initial_state.setZero();
    DeclareDiscreteState(initial_state);

    DeclareVectorOutputPort("command", nq_ + nv_, &HardwarePlant::CommandOutput, {this->all_state_ticket()});
    DeclareVectorOutputPort("state", nq_ + nv_, &HardwarePlant::StateOutput, {this->all_state_ticket()});
    DeclarePeriodicDiscreteUpdateEvent(dt_, 0, &HardwarePlant::Update);
  }

  void HardwarePlant::Initial(systems::Context<double> &context, Eigen::VectorXd state) const
  {
    context.get_mutable_discrete_state(0).set_value(state);
  }

  void HardwarePlant::Update(const systems::Context<double> &context, systems::DiscreteValues<double> *next_state) const
  {
    static struct timespec t0, t1, last_time;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    // qv << q, v;
    // next_state->set_value(qv);
    
    csp_test();

    /* 泄力 */
    // joint_cmd[0].torqueOffset = 0;
    // joint_cmd[0].torque = 0.0;
    // joint_cmd[0].maxTorque = MAX_CURRENT;
    // actuators.setJointTorque(joint_ids, na_, joint_cmd);

    clock_gettime(CLOCK_MONOTONIC, &t1);
#if 0
    printf("Update time: %f\n", TIME_DIFF(t0, t1) * 1000);
    printf("Realtime period: %f\n\n", TIME_DIFF(last_time, t1) * 1000);
    last_time = t1;
#endif
  }

  void HardwarePlant::CommandOutput(const systems::Context<double> &context, systems::BasicVector<double> *output) const
  {
    output->SetFromVector(context.get_discrete_state(0).value());
  }

  void HardwarePlant::StateOutput(const systems::Context<double> &context, systems::BasicVector<double> *output) const
  {
    Eigen::VectorXd q(nq_), v(nv_), vdot(nv_), tau(na_);
    Eigen::VectorXd qv(nq_ + nv_);

    readJoint(na_, dt_, q, v, vdot, tau);
    q = q / 3.14159 * 180.0;
    v = v / 3.14159 * 180.0;
    qv << q, v;

    output->SetFromVector(qv);
  }
}; // namespace drake

void csp_test()
{
  Eigen::VectorXd q(1), v(1), vdot(1), tau(1);
  Eigen::VectorXd qv(1 + 1);
  readJoint(1, FLAGS_dt_, q, v, vdot, tau);
  
  /* 常量 */
  const double g = 9.8; // gravity
  const double rad2deg = 180./M_PI;
  const double w = 2 * M_PI;
  const double dt = FLAGS_dt_;
  const double t_max = 1.0;

  /* 时间辍 */
  static int count = 0;
  static double t = 0;
  count ++;
  t = count * dt;

  /* 模型 */
  const double m1 = 0.5; // kg
  const double l1 = 0.25 + 0.04; //m
  const double m2 = 0.65; // kg
  const double l2 = 0.5 + 0.04; //m
  const double m3 = 0.06;
  const double l3 = 0.02;
  const double I = m1*l1*l1 + m2*l2*l2 + m3*l3*l3; //0.206

  /* 配置 */
  const double A = 10;  //摆动位置幅度(单位d)
  const double T = 0.25;

  /* 轨迹 */
  double pos;
  double vel;
  double acc;
  if(t > t_max)
  {
    pos = 90.0;
    vel = 0;
    acc = 0;
  }
  else 
  {
    pos = -363.41 * t * t * t * t * t \
          +1116.4 * t * t * t * t \
          -1349.4 * t * t * t \
          +692.32 * t * t \
          -6.4006 * t \
          +0.0545;
    vel = -363.41 * 5 * t * t * t * t \
          +1116.4 * 4 * t * t * t \
          -1349.4 * 3 * t * t \
          +692.32 * 2 * t \
          -6.4006 * 1;
    acc = -363.41 * 5 * 4 * t * t * t \
          +1116.4 * 4 * 3 * t * t \
          -1349.4 * 3 * 2 * t \
          +692.32 * 2 * 1;
  }

  /* 状态前馈 */
  double torque_fd = (m1*l1+m2*l2+m3*l3)*g*cos(joint_data[0].position/rad2deg);
  // double torque_fd = (m1*l1+m2*l2+m3*l3)*g*cos(pos/rad2deg);

  /* 运动前馈 */
  double torque_move_fd = acc / rad2deg * I;

  /* 打印 */
  // joint_cmd[0].torque = torque_fd + torque_move_fd;

  /* 输出 */
  // joint_cmd_old[0] = joint_cmd[0];
  // joint_cmd[0].position = pos;
  // joint_cmd[0].velocityOffset = vel;
  // joint_cmd[0].torqueOffset = (torque_fd + torque_move_fd) / 1.44;
  // joint_cmd[0].maxTorque = MAX_TORQUE;
  // EM_setPositions(joint_ids, 1, joint_cmd);

  /* 泄力 */
  joint_cmd[0].torqueOffset = 0;
  joint_cmd[0].torque = 0;
  joint_cmd[0].maxTorque = 0;
  actuators.setJointTorque(joint_ids, 1, joint_cmd);

  /* 仅重力补偿 */
  // joint_cmd[0].torque = torque_fd / 1.44;
  // joint_cmd[0].maxTorque = MAX_TORQUE;
  // actuators.setJointTorque(joint_ids, 1, joint_cmd);
}

