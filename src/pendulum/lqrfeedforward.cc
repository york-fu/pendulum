#include "lqrfeedforward.h"
#include <iostream>
#include <fstream>

using namespace std;

namespace drake {
    lqrfeedforward::lqrfeedforward(multibody::MultibodyPlant<double> *plant, 
                    pendulum::PendulumParameters parameters) : plant_(plant)
    {
        na_ = plant_->num_actuated_dofs();
        nq_ = plant_->num_positions();
        nv_ = plant_->num_velocities();
        dt_ = plant_->time_step();
        printf("nq: %d\n", nq_);
        T = T_f / dt_;
        DeclareVectorInputPort("input", nq_+nv_);  //输入
        DeclareVectorOutputPort("output", na_, &lqrfeedforward::OUT);  //输出
        DeclareVectorOutputPort("comoutput", nq_+nv_, &lqrfeedforward::comOUT);  //输出

        m_ = parameters.m();  //质量
        g_ = parameters.g();  //重力加速度
        l_ = parameters.l();  //杆长

        prev_v_.resize(nv_);
        i.resize(nv_);
        q_desire.resize(nq_);
        v_desire.resize(nv_);
        vd_desire.resize(nv_);
        q_measure.resize(nq_);
        v_measure.resize(nv_);
        vd_measure.resize(nv_);
        q_err.resize(nq_);
        v_err.resize(nv_);
        vd_err.resize(nv_);
        pout.resize(nq_);
        iout.resize(nv_);
        dout.resize(nv_);
        forout.resize(pout.size());
        f.resize(pout.size());
        out_ll.resize(pout.size());
        out_ll.setZero();
        out_l.resize(pout.size());
        out_l.setZero();
        out.resize(pout.size());
        out.setZero();
    }
    
    void lqrfeedforward::OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const
    {
        Eigen::VectorBlock<VectorX<double>> output_vector = output->get_mutable_value();
        output_vector.setZero();

        double A = 0.5;  //幅度
        q_desire << M_PI;// + A * sin(pi2 / T_f * t_f);
        v_desire << 0;//A * pi2 / T_f * cos(pi2 / T_f * t_f);
        vd_desire << 0;//A * pi2 / T_f *pi2 / T_f * -sin(pi2 / T_f * t_f);

        const auto qv = get_input_port().Eval(context);
        q_measure = qv.segment(0, nq_);  //位置（仿真是rad）
        v_measure = qv.segment(nq_, nv_);  //速度（仿真是rad/s）
        vd_measure = (v_measure - prev_v_) / dt_;  //加速度(仿真是rad/s2)
        prev_v_ = v_measure;

        q_err = q_desire - q_measure;
        v_err = v_desire - v_measure;
        vd_err = vd_desire - vd_measure;
        // i += (q_err + v_err);

        /* 位置速度pp控制 */
        pout = kp * q_err;
        dout = kd * v_err;

        // forout << m_ * g_ * l_ * sin(q_desire(0) - theta_mid + 3.14159);  //补偿目标重力
        forout << m_ * g_ * l_ * sin(q_measure(0));  //补偿当前重力产生的扭矩

        out = pout + dout;// + forout;

        double max_torque = 44.14;
        if(out(0) > max_torque)
        {
            out(0) = max_torque;
        }
        else if(out(0) < -max_torque)
        {
            out(0) = -max_torque;
        }
        
        if(v_measure(0) < 0)
        {
            f(0) = 1;
        }
        else if(v_measure(0) > 0)
        {
            f(0) = -1;
        }
        else if(v_measure(0) == 0)
        {
            if(v_desire(0) < 0)
            {
                f(0) = 2;
            }
            else if(v_desire(0) > 0)
            {
                f(0) = -2;
            }
        }
        // out(0) += f(0);

        output_vector << out_ll;
        out_ll = out_l;
        out_l = out;
        // output_vector << 0;
        // printf("%f\r", out(0));

        // FILE* file;
        // file = fopen("/home/chen/Desktop/test.txt", "a");
        // double temp = (q_measure(0) / 3.14159 * 180.0);
        // fprintf(file, "%4d %.7f %.7f %7f\n", t2, temp, v_measure(0), out(0));
        
        t++;
        t %= T;
        t2++;
        t_f = t * dt_;
        t2_f = t2 * dt_;

        /*
        执行10000ms A=0.5 T=1000ms P=93.7263 D=7.3775 m=1
        有前馈 39.170402
        无前馈 63.025596

        执行10000ms A=0.5 T=1000ms P=979.846 D=220.531 m=100
        有前馈 404.937196
        无前馈 607.260709

        执行10000ms A=0.5 T=1000ms P=979.846 I=100 D=220.531 m=100  位置速度积分
        有前馈 33.368437
        无前馈 47.883404

        执行10000ms A=0.5 T=500ms P=979.846 I=100 D=220.531 m=100
        有前馈 1528.264459
        无前馈 1512.863225

        执行10000ms A=0.5 T=500ms P=979.846 D=220.531 m=100
        有前馈 1748.229525
        无前馈 1794.222598

        执行10000ms A=0.5 T=1000ms P=979.846 I=100 D=220.531 m=100 l=0.5  位置积分
        有前馈 发散
        无前馈 发散

        执行10000ms A=0.5 T=1000ms P=979.846 I=100 D=220.531 m=100 l=0.5  速度积分
        有前馈 34.592715
        无前馈 48.744127
        */
    }

    void lqrfeedforward::comOUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const
    {
        Eigen::VectorBlock<VectorX<double>> output_vector = output->get_mutable_value();

        auto qv = get_input_port().Eval(context);

        qv[0] = q_desire(0);
        qv[1] = v_desire(0);

        // printf("%f %f\r",qv[0], qv[1]);

        output_vector = qv;
    }

}  // namespace drake