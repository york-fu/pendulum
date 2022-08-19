#include "lqrfeedforward.h"
#include <iostream>
#include <fstream>

namespace drake {

    lqrfeedforward::lqrfeedforward(multibody::MultibodyPlant<double> *plant, 
                    pendulum::PendulumParameters parameters) : plant_(plant)
    {
        na_ = plant_->num_actuated_dofs();
        nq_ = plant_->num_positions();
        nv_ = plant_->num_velocities();
        dt_ = plant_->time_step();
        DeclareVectorInputPort("input", nq_+nv_);  //输入
        DeclareVectorOutputPort("output", na_, &lqrfeedforward::OUT);  //输出
        DeclareVectorOutputPort("comoutput", nq_+nv_, &lqrfeedforward::comOUT);  //输出

        m_ = parameters.m();  //质量
        g_ = parameters.g();  //重力加速度
        l_ = parameters.l();  //杆长
    }
    
    void lqrfeedforward::OUT(const systems::Context<double>& context, systems::BasicVector<double>* output) const
    {
        Eigen::VectorBlock<VectorX<double>> output_vector = output->get_mutable_value();
        Eigen::VectorBlock<VectorX<double>> static_vector = output->get_mutable_value();

        output_vector.setZero();
        static_vector.setZero();

        const auto qv = get_input_port().Eval(context);

        q_measure = qv.segment(0, nq_);  //位置(度)
        v_measure = qv.segment(nq_, nv_);  //速度（dps）
        vd_measure.resize(v_measure.size());
        prev_v_.resize(v_measure.size());
        vd_measure = (v_measure - prev_v_) / dt_;  //加速度(dps2)

        q_desire.resize(q_measure.size());
        v_desire.resize(v_measure.size());
        vd_desire.resize(vd_measure.size());

        static int32_t t = 0;
        static int32_t t2 = 0;
        int32_t T = 1000;
        double w = 2 * 3.14159;
        double A = 0.5;


        q_desire << 3.14159 + A * sin((double)t / (double)T * w);
        v_desire << A * w * cos((double)t / (double)T * w);
        vd_desire << 0;
        t++;
        t2++;
        t %= T;

        printf("%d, %lf\r",t2, err_sum);
        // if(t2 <= 10000)
        // {
        //     std::ofstream file;
        //     file.open("/home/chen/test.txt", std::ios::out | std::ios::app);
        //     file << t << " " << q_desire(0) << " " << q_measure(0) << std::endl;
        //     file.close();
        // }

        if(t2 <= 10000)
            err_sum += fabs(q_desire(0) - q_measure(0));

        Eigen::VectorXd q_err, v_err, vd_err;  //误差位置、速度、加速度

        q_err = q_desire - q_measure;
        v_err = v_desire - v_measure;
        vd_err = vd_desire - vd_measure;

        Eigen::VectorXd pout, iout, dout;  //pid输出

        static Eigen::VectorXd i;  //积分量

        i.resize(q_err.size());

        Eigen::VectorXd kp = Eigen::VectorXd::Constant(1, 10);
        Eigen::VectorXd ki = Eigen::VectorXd::Constant(1, 0);
        Eigen::VectorXd kd = Eigen::VectorXd::Constant(1, 4);

        pout = kp * (q_err + v_err);
        i += (q_err + v_err);
        iout = ki * i;
        dout = kd * (q_err + v_err);

        Eigen::VectorXd forout;  //前馈输出
        forout.resize(pout.size());
        forout << m_ * g_ * l_ * sin(q_desire(0));  //正数逆时针

        Eigen::VectorXd out = pout + iout + dout + forout;

        output_vector << out;

        //执行10000ms A=0.5 T=1000ms P=93.7263 D=7.3775 m=1
        //有前馈 39.170402
        //无前馈 63.025596

        //执行10000ms A=0.5 T=1000ms P=979.846 D=220.531 m=100
        //有前馈 404.937196
        //无前馈 607.260709

        //执行10000ms A=0.5 T=1000ms P=979.846 I=100 D=220.531 m=100  位置速度积分
        //有前馈 33.368437
        //无前馈 47.883404

        //执行10000ms A=0.5 T=500ms P=979.846 I=100 D=220.531 m=100
        //有前馈 1528.264459
        //无前馈 1512.863225

        //执行10000ms A=0.5 T=500ms P=979.846 D=220.531 m=100
        //有前馈 1748.229525
        //无前馈 1794.222598

        //执行10000ms A=0.5 T=1000ms P=979.846 I=100 D=220.531 m=100 l=0.5  位置积分
        //有前馈 发散
        //无前馈 发散

        //执行10000ms A=0.5 T=1000ms P=979.846 I=100 D=220.531 m=100 l=0.5  速度积分
        //有前馈 34.592715
        //无前馈 48.744127
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