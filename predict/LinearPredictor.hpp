//
// Created by Haoran-Jiang on 2021/11/25.
//

#ifndef PREDICT_LINEAR_PREDICTOR_H
#define PREDICT_LINEAR_PREDICTOR_H

// modules
#include "predict.hpp"
#include "common.hpp"
#include "kalman.h"
#include "extended_kalman_filter.hpp"

// packages
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace predict
{
    class LinearPredictor
    {
    private:
        bool last_track{false};   // 上一次是否有追踪目标
        Pos3D last_pw;            // 上一次世界坐标
        bbox_t last_bbox;         // 上一次预测框
        double comm_latency;      // 通讯相关延迟
        ExtendedKalmanFilter ekf; // 濾波器

        std::chrono::steady_clock::time_point last_t; // 用於計算dt_，以更新濾波器中的轉移矩陣等
        double dt_;
        Eigen::VectorXd target_state;
        double last_yaw_;

        int detect_count_;
        int lost_count_;

        double s2qxyz_, s2qyaw_, s2qr_;
        double r_xyz_factor, r_yaw;

    public:
        // To store another pair of armors message
        double last_z, last_r;

        explicit LinearPredictor(double latency = .020)
        {
            last_t = std::chrono::steady_clock::now();
            tracker_state = LOST;
            last_track = false;
            comm_latency = latency;
            // EKF
            // xa = x_armor, xc = x_robot_center
            // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
            // measurement: xa, ya, za, yaw
            // f - Process function
            auto f = [this](const Eigen::VectorXd &x)
            {
                Eigen::VectorXd x_new = x;
                x_new(0) += x(1) * dt_;
                x_new(2) += x(3) * dt_;
                x_new(4) += x(5) * dt_;
                x_new(6) += x(7) * dt_;
                return x_new;
            };
            // J_f - Jacobian of process function
            auto j_f = [this](const Eigen::VectorXd &)
            {
                Eigen::MatrixXd f(9, 9);
                // clang-format off
                f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
                      0,   1,   0,   0,   0,   0,   0,   0,   0,
                      0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
                      0,   0,   0,   1,   0,   0,   0,   0,   0,
                      0,   0,   0,   0,   1,   dt_, 0,   0,   0,
                      0,   0,   0,   0,   0,   1,   0,   0,   0,
                      0,   0,   0,   0,   0,   0,   1,   dt_, 0,
                      0,   0,   0,   0,   0,   0,   0,   1,   0,
                      0,   0,   0,   0,   0,   0,   0,   0,   1;
                // clang-format on
                return f;
            };
            // h - Observation function
            auto h = [](const Eigen::VectorXd &x)
            {
                Eigen::VectorXd z(4);
                double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
                z(0) = xc - r * cos(yaw); // xa
                z(1) = yc - r * sin(yaw); // ya
                z(2) = x(4);              // za
                z(3) = x(6);              // yaw
                return z;
            };
            // J_h - Jacobian of observation function
            auto j_h = [](const Eigen::VectorXd &x)
            {
                Eigen::MatrixXd h(4, 9);
                double yaw = x(6), r = x(8);
                // clang-format off
                //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
                h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
                      0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
                      0,   0,   0,   0,   1,   0,   0,          0,   0,
                      0,   0,   0,   0,   0,   0,   1,          0,   0;
                // clang-format on
                return h;
            };
            // update_Q - process noise covariance matrix
            cv::FileStorage ekf_param("../ekf.yml", cv::FileStorage::READ);
            s2qxyz_ = ekf_param["s2qxyz_"].real();
            s2qyaw_ = ekf_param["s2qyaw_"].real();
            s2qr_ = ekf_param["s2qr_"].real();
            auto u_q = [this]()
            {
                Eigen::MatrixXd q(9, 9);
                double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
                double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
                double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
                double q_r = pow(t, 4) / 4 * r;
                // clang-format off
                //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
                q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
                      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
                      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
                      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
                      0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
                      0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
                      0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
                      0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
                      0,      0,      0,      0,      0,      0,      0,      0,      q_r;
                // clang-format on
                return q;
            };
            // update_R - measurement noise covariance matrix
            r_xyz_factor = ekf_param["r_xyz_factor"].real();
            r_yaw = ekf_param["r_yaw"].real();
            auto u_r = [this](const Eigen::VectorXd &z)
            {
                Eigen::DiagonalMatrix<double, 4> r;
                double x = r_xyz_factor;
                r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
                return r;
            };
            // P - error estimate covariance matrix
            Eigen::DiagonalMatrix<double, 9> p0;
            p0.setIdentity();
            ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
            /// 初始化count
            detect_count_ = 0;
            lost_count_ = 0;
            ekf_param.release();
        };
        // 状态机
        enum State
        {
            LOST,
            DETECTING,
            TRACKING,
            TEMP_LOST,
        } tracker_state;
        void predict(std::shared_ptr<ThreadDataPack> &data, PositionTransform &position_transform);
        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x);
        double orientationToYaw(const bbox_t armor, PositionTransform &position_transform);
        void initEKF(const Pos3D &a, const bbox_t armor, PositionTransform &position_transform);
        Eigen::Vector3d predict_armor_after_t(const Eigen::VectorXd &target_state, double t);
        double minimumAngleDifference(double angle1, double angle2);
        void handleArmorJump(const Pos3D &a, const bbox_t armor, PositionTransform &position_transform);
        Eigen::Vector2d getArmorVectorFromState(const Eigen::VectorXd &x);

        // for debug
        inline Eigen::VectorXd get_target_state() { return target_state; };
        void print_tracker_state()
        {
            switch (tracker_state)
            {
            case LOST:
                LOGM_S("[TRACKER] LOST");
                break;
            case DETECTING:
                LOGM_S("[TRACKER] DETECTING");
                break;
            case TRACKING:
                LOGM_S("[TRACKER] TRACKING");
                break;
            case TEMP_LOST:
                LOGM_S("[TRACKER] TEMP_LOST");
                break;
            default:
                LOGM_S("[TRACKER] shit");
                break;
            }
        };
    };
}

#endif // PREDICT_LINEAR_PREDICTOR_H