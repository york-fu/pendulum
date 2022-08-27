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
            multibody::MultibodyPlant<double> *plant_;
            int32_t na_;  //驱动个数
            int32_t nq_;  //位置个数
            int32_t nv_;  //速度个数
            double dt_;  //时间间隔
            double m_;  //质量
            double g_;  //重力加速度
            double l_;  //杆长
            mutable Eigen::VectorXd prev_v_;  //上一时刻速度
            mutable Eigen::VectorXd i;  //积分量
            mutable Eigen::VectorXd q_desire, v_desire, vd_desire;  //目标位置、速度、加速度
            mutable Eigen::VectorXd q_measure, v_measure, vd_measure;  //状态位置、速度、加速度
            mutable Eigen::VectorXd q_err, v_err, vd_err;  //误差位置、速度、加速度
            mutable Eigen::VectorXd pout, iout, dout;  //pid输出
            mutable Eigen::VectorXd forout;  //前馈输出
            mutable Eigen::VectorXd f;  //阻力
            mutable Eigen::VectorXd out_ll;  //上上个输出
            mutable Eigen::VectorXd out_l;  //上个输出
            mutable Eigen::VectorXd out;  //输出
            mutable int32_t t = 0;
            mutable int32_t t2 = 0;
            mutable int32_t T;
            mutable double t_f = 0;  //0~T的时间(s)
            mutable double t2_f = 0;  //总时间(s)
            double T_f = 1;  //周期(s)
            double pi2 = 2 * M_PI;
            Eigen::VectorXd kp = Eigen::VectorXd::Constant(1, 1500);  //4.44835 94.9746
            Eigen::VectorXd ki = Eigen::VectorXd::Constant(1, 0);
            Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 3.7893);  //3.04723 3.7893
    };


}  // namespace drake


#endif