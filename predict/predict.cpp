//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "predict.hpp"

namespace predict
{
    void Predict::operator()(autoaim_pipline &pipbefore, autoaim_pipline &pipafter)
    {
        /**
         * @brief 检查类是否正确初始化
         */
        if (!_init)
        {
            LOGE_S("[predict]Error: run before init.");
            return;
        }

        LOGM_S("[predict] running");

        LinearPredictor _predictor(comm_latency);

        do
        {
            auto obj = pipbefore.get(); /*!< 从上一线程的缓存队列获取报文指针 */
            cv::Mat img_show;

            _predictor.predict(obj, position_transform);

            if (_debug)
            {
                auto &send = obj->robotcommand;
                LOGM_S("[predict] pitch %6.2f, yaw %6.2f, dist %4.1f",
                       send.pitch_angle, send.yaw_angle,
                       (float)send.distance / 10);
            }
            if (_show)
            {
                Eigen::VectorXd target_state = _predictor.get_target_state();
                LOGM_S("[EKF] xc %.2f, yc %.2f, za %.2f, theta %.2f, Vxc %.2f, Vyc %.2f, Vza %.2f, Vtheta %.2f, z %.2f",
                       target_state(0), target_state(1), target_state(2),
                       target_state(3), target_state(4), target_state(5),
                       target_state(6), target_state(7), target_state(8));
            }

            pipafter.put(obj); /*!< 向下一线程的缓存队列提交报文指针*/
        } while (_run);
        LOGM_S("[predict] stop");
    }
}
