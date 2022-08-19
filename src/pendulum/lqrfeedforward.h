#ifndef __LQRFEEDFORWARD_H
#define __LQRFEEDFORWARD_H

#include "drake/multibody/plant/multibody_plant.h"
#include <drake/multibody/benchmarks/pendulum/make_pendulum_plant.h>
#include "make_pendulum_plant.h"

namespace drake {

    class lqrfeedforward : public systems::LeafSystem<double>
    {
        public:
            DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(lqrfeedforward)

            lqrfeedforward(multibody::MultibodyPlant<double> *plant, 
                           pendulum::PendulumParameters parameters);
        
        private:
            void OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const;
            void comOUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const;
            int32_t na_;  //驱动个数
            int32_t nq_;  //位置个数
            int32_t nv_;  //速度个数
            double m_;  //质量
            double g_;  //重力加速度
            double l_;  //杆长
            double dt_;  //时间间隔
            mutable Eigen::VectorXd q_measure, v_measure, vd_measure;  //位置、速度、加速度
            mutable Eigen::VectorXd prev_v_;  //上一时刻速度
            mutable double err_sum = 0;  //误差和
            multibody::MultibodyPlant<double> *plant_;
            mutable Eigen::VectorXd q_desire, v_desire, vd_desire;  //目标位置、速度、加速度
    };


}  // namespace drake


#endif