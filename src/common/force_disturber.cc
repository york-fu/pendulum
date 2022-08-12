#include "force_disturber.h"

namespace drake
{
  ForceDisturber::ForceDisturber(multibody::BodyIndex bodyIndex, Eigen::Matrix<double, 6, 1> forceVec,
                                 double startTime, double disturbDuration, double disturbPeriod) : _bodyIndex{bodyIndex},
                                                                                                   _startTime{startTime},
                                                                                                   _disturbDuration{disturbDuration},
                                                                                                   _disturbPeriod{disturbPeriod}
  {

    _forceVec = forceVec;
    DeclareAbstractOutputPort("spatial_forces_vector", &ForceDisturber::Output);
  }

  void ForceDisturber::Output(const systems::Context<double> &context,
                              std::vector<multibody::ExternallyAppliedSpatialForce<double>> *result) const
  {
    static uint32_t count = 1;
    static double triggerTime = _startTime;
    const double &currTime = context.get_time();
    Eigen::Vector3d pos(0, 0, 0);
    Eigen::Vector3d tau(0, 0, 0);
    Eigen::Vector3d f(0, 0, 0);
    result->resize(1);
    (*result)[0].body_index = _bodyIndex;

    if ((currTime >= triggerTime) && (currTime < triggerTime + _disturbDuration))
    {
      pos << 0, 0, 0;
      tau = _forceVec.block<3, 1>(0, 0);
      f = _forceVec.block<3, 1>(3, 0);
      (*result)[0].F_Bq_W = multibody::SpatialForce<double>(tau, f);
      (*result)[0].p_BoBq_B = pos;
    }
    else
    {
      pos << 0, 0, 0;
      tau << 0, 0, 0;
      f << 0, 0, 0;
      (*result)[0].F_Bq_W = multibody::SpatialForce<double>(tau, f);
      (*result)[0].p_BoBq_B = pos;
    }

    if (fabs(currTime - (triggerTime + _disturbDuration)) < 1e-2)
    {
      triggerTime += _disturbPeriod;
    }
  }
} // namespace drake
