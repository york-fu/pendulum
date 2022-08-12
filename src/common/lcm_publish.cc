#include "lcm_publish.h"

void lcmPublishValue(lcm::LCM *lc, std::string name, const double value)
{
  lcm_std_msgs::Float64 f64Msg;
  f64Msg.data = value;
  lc->publish(name, &f64Msg);
}

void lcmPublishVector(lcm::LCM *lc, std::string name, const Eigen::VectorXd vec)
{
  lcm_std_msgs::Float64MultiArray f64ArrayMsg;
  f64ArrayMsg.size = vec.size();
  f64ArrayMsg.data.resize(f64ArrayMsg.size);
  f64ArrayMsg.data.assign(&vec[0], vec.data() + f64ArrayMsg.size);
  lc->publish(name, &f64ArrayMsg);
}

void lcmPublishState(lcm::LCM *lc, std::string prefix,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &v, const Eigen::VectorXd &vdot,
                     bool to_rpy)
{
  lcm_std_msgs::Float64MultiArray f64ArrayMsg;
  if (to_rpy)
  {
    f64ArrayMsg.size = q.size() - 1;
    f64ArrayMsg.data.resize(f64ArrayMsg.size);
    f64ArrayMsg.data.assign(&q[1], q.data() + q.size());
    Eigen::Vector3d eulerAngle = drake::math::RollPitchYawd(Eigen::Quaternion(q[0], q[1], q[2], q[3])).vector();
    for (uint32_t i = 0; i < 3; i++)
    {
      f64ArrayMsg.data[i] = eulerAngle[i];
    }
  }
  else
  {
    f64ArrayMsg.size = q.size();
    f64ArrayMsg.data.resize(f64ArrayMsg.size);
    f64ArrayMsg.data.assign(&q[0], q.data() + q.size());
  }
  lc->publish(prefix + "/q", &f64ArrayMsg);

  f64ArrayMsg.size = v.size();
  f64ArrayMsg.data.resize(f64ArrayMsg.size);
  f64ArrayMsg.data.assign(&v[0], v.data() + v.size());
  lc->publish(prefix + "/v", &f64ArrayMsg);

  f64ArrayMsg.size = vdot.size();
  f64ArrayMsg.data.resize(f64ArrayMsg.size);
  f64ArrayMsg.data.assign(&vdot[0], vdot.data() + vdot.size());
  lc->publish(prefix + "/vdot", &f64ArrayMsg);
}
