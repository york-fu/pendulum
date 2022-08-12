#include "hardware_plant.h"
#include <gflags/gflags.h>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <mutex>
#include "utils.h"
#include "lcm_publish.h"

DECLARE_bool(pub);

#define NUM_JOINT_MAX 12
#define BIT_17 (1 << 17)
#define BIT_17_9 (BIT_17 * 9)
#define C2T_COEFFICIENT (1.44)
#define MAX_TORQUE (44.14)
#define MAX_CURRENT (MAX_TORQUE / C2T_COEFFICIENT)

const char *ifname = "enp2s0";
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
  std::vector<double> T(num_joint, 0);
  for (uint32 i = 0; i < num_joint; i++)
  {
    T[i] = (goal_pos[i] - joint_data[i].position) / speed;
    printf("Joint %d from %f to %f\n", i + 1, joint_data[i].position, goal_pos[i]);
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
      joint_cmd[i].position = calcCos(joint_data[i].position, goal_pos[i], total_cnt, count);
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
                      Eigen::VectorXd joint_q,
                      Eigen::VectorXd joint_v,
                      Eigen::VectorXd joint_vd,
                      Eigen::VectorXd joint_tau)
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

static void writeJoint(uint32_t na, Eigen::VectorXd cmd)
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

  std::vector<double> inital_pos(NUM_JOINT_MAX, 0);
  jointMoveTo(inital_pos, 60, 1.0e-3);
  return 0;
}

void HardwarePlantDeInit()
{
  std::vector<double> goal_pos(NUM_JOINT_MAX, 0);
  jointMoveTo(goal_pos, 60, 1.0e-3);
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
    dt_ = plant_->time_step();
    Eigen::VectorXd initial_state(nq_ + nv_);
    initial_state.setZero();
    DeclareDiscreteState(initial_state);

    DeclareVectorInputPort("actuation", na_);
    DeclareVectorOutputPort("state", nq_ + nv_, &HardwarePlant::Output, {this->all_state_ticket()});
    actuation_cache_ = &DeclareCacheEntry("actuation_cache", systems::BasicVector<double>(na_), &HardwarePlant::ActuationCache,
                                          {get_actuation_input_port().ticket()});
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

    Eigen::VectorXd q(nq_), v(nv_), vdot(nv_), tau(na_);
    Eigen::VectorXd qv(nq_ + nv_);

    readJoint(na_, dt_, q, v, vdot, tau);
    qv << q, v;
    next_state->set_value(qv);

    auto actuation = actuation_cache_->Eval<systems::BasicVector<double>>(context).get_value();
    writeJoint(na_, actuation);

    lcmPublishState(lcm_ptr, "state", q, v, vdot, false);
    lcmPublishVector(lcm_ptr, "state/tau", tau);

    clock_gettime(CLOCK_MONOTONIC, &t1);
#if 0
    printf("Update time: %f\n", TIME_DIFF(t0, t1) * 1000);
    printf("Realtime period: %f\n\n", TIME_DIFF(last_time, t1) * 1000);
    last_time = t1;
#endif
  }

  void HardwarePlant::ActuationCache(const systems::Context<double> &context, systems::BasicVector<double> *output) const
  {
    output->SetFromVector(get_actuation_input_port().Eval(context));
  }

  void HardwarePlant::Output(const systems::Context<double> &context, systems::BasicVector<double> *output) const
  {
    output->SetFromVector(context.get_discrete_state(0).value());
  }
}; // namespace drake
