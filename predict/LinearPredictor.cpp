//
// Created by Haoran-Jiang on 2021/11/25.
//

#include "LinearPredictor.hpp"

namespace predict
{

    /// 选择高度限制
    constexpr double height_thres = -20.;
    /// 识别双阈值
    constexpr float conf_thres = 0.6f;
    /// 远距离弹量控制
    constexpr double distant_threshold = 6.;
    // Detecting->Tracking阈值
    constexpr int tracking_threshold_ = 5;
    // Temp_Lost->Lost 阈值
    constexpr int lost_threshold_ = 5;
    // max_match_distance
    constexpr double max_match_distance_ = 0.5;

    double LinearPredictor::orientationToYaw(const bbox_t armor, PositionTransform &position_transform){
        double yaw = position_transform.pnp_get_armor_angle(armor.pts, armor.tag_id);

        // // set yaw range in [-pi/2, pi/2]
        // if(yaw > M_PI / 2)  yaw -= M_PI;
        // else if(yaw < M_PI / 2) yaw += M_PI;

        yaw = last_yaw_ + minimumAngleDifference(last_yaw_, yaw);
        last_yaw_ = yaw;
        std::cout << "yaw: " << yaw << std::endl;
        return yaw;
    }

    Eigen::Vector3d LinearPredictor::getArmorPositionFromState(const Eigen::VectorXd &x)
    {
        // Calculate predicted position of the current armor
        double xc = x(0), yc = x(2), zc = x(4);
        double yaw = x(6), r = x(8);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        return Eigen::Vector3d(xa, ya, zc);
    }

    Eigen::Vector2d LinearPredictor::getArmorVectorFromState(const Eigen::VectorXd &x){
        // Calculate predicted position of the current armor
        double Vxc = x(1), Vyc = x(3);
        double Vyaw = x(7);
        double Vxa = Vxc - cos(Vyaw);
        double Vya = Vyc - sin(Vyaw);
        return Eigen::Vector2d(Vxa, Vya);
    }

    double LinearPredictor::minimumAngleDifference(double angle1, double angle2)
    {
        double difference = angle2 - angle1;
        difference = std::fmod(difference + M_PI, 2.0 * M_PI);
        if (difference < 0)
            difference += 2.0 * M_PI;
        return difference - M_PI;
    }

    void LinearPredictor::initEKF(const Pos3D &a, const bbox_t armor, PositionTransform &position_transform)
    {
        double xa = a(0);
        double ya = a(1);
        double za = a(2);
        last_yaw_ = 0;
        double yaw = orientationToYaw(armor, position_transform);

        // Set initial position at 0.2m behind the target
        target_state = Eigen::VectorXd::Zero(9);
        double r = 0.2;
        double xc = xa + r * cos(yaw);
        double yc = ya + r * sin(yaw);
        double zc = za;
        last_z = zc, last_r = r;
        target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

        ekf.setState(target_state);
    }

    void LinearPredictor::handleArmorJump(const Pos3D &a, const bbox_t armor, PositionTransform &position_transform)
    {
        double last_yaw = target_state(6);
        double yaw = orientationToYaw(armor, position_transform);

        if (abs(yaw - last_yaw) > 0.4)
        {
            last_z = target_state(4);
            target_state(4) = a(2);
            target_state(6) = yaw;
            std::swap(target_state(8), last_r);
        }

        auto p = a;
        Eigen::Vector3d current_p(p(0), p(1), p(2));
        Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

        if ((current_p - infer_p).norm() > max_match_distance_)
        {
            double r = target_state(8);
            target_state(0) = p(0) + r * cos(yaw);  // xc
            target_state(1) = 0;                   // vxc
            target_state(2) = p(1) + r * sin(yaw);  // yc
            target_state(3) = 0;                   // vyc
            target_state(5) = 0;                   // vza
        }

        ekf.setState(target_state);
    }

    Eigen::Vector3d LinearPredictor::predict_armor_after_t(const Eigen::VectorXd &target_state, double t)
    {
        double xc = target_state(0) + t * target_state(4);
        double yc = target_state(1) + t * target_state(5);
        double yaw = target_state(3) + t * target_state(7);
        double za = target_state(2);
        double Vza = target_state(8);
        Eigen::VectorXd predict_target_state(9);
        predict_target_state << xc, yc, za, yaw, 0.0, 0.0, 0.0, 0.0, Vza;
        return getArmorPositionFromState(predict_target_state);
    }

    void LinearPredictor::predict(std::shared_ptr<ThreadDataPack> &data, PositionTransform &position_transform)
    {
        // print_tracker_state();
        auto &detections = data->bboxes;
        auto q_raw = data->attitude.toQuaternion();
        auto &img = data->frame;
        auto t = data->time;
        auto &send = data->robotcommand;
        auto robot_status = data->robotstatus;

        Eigen::Quaternionf q(q_raw.matrix().transpose()); // 重建四元数
        Eigen::Matrix3d R_IW = q.matrix().cast<double>(); // 生成旋转矩阵
        position_transform.update_R_IW(R_IW);             // 更新旋转矩阵

        bbox_t armor;

        /// 过滤出敌方颜色的装甲板 && 判断是否有英雄出现
        std::vector<bbox_t> new_detections; // new_detection: vector 是经过过滤后所有可能考虑的装甲板
        for (auto &d : detections)
        {
            if ((int(robot_status.enemy_color) == d.color_id && d.tag_id != 0) ||
                (int(robot_status.enemy_color) == 2)) // 不能随意修改，否则会数组越界0-5
            {
                /* 放行正确颜色的装甲板 */
                Pos3D m_pc = position_transform.pnp_get_pc(d.pts, d.tag_id); // point camera: 目标在相机坐标系下的坐标
                Pos3D m_pw = position_transform.pc_to_pw(m_pc);              // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                if (m_pw[2] < height_thres)
                {
                    LOGW_S("To High! height is %lf", m_pw[2]);
                    continue;
                }
                if (int(robot_status.game_state) == 0)
                {
                    double distance = m_pw.norm();
                    if (distance > distant_threshold)
                    {
                        LOGW_S("To Far! Distance is %lf", distance);
                        continue;
                    }
                }

                if (d.confidence >= conf_thres)
                    /* 阈值大于 conf_thres 直接放行 */
                    new_detections.push_back(d);
            }
        }

        // if (tracker_state == TRACKING || tracker_state == TEMP_LOST) // 寻找上一次打击的装甲板
        // {
        //     // LOGM_S("[linear] Try Armor");
        //     for (auto &d : new_detections)
        //     {
        //         auto center = points_center(d.pts);
        //         Pos3D pw = position_transform.pnp_get_pw(d.pts, d.tag_id);
        //         if (last_track && (center.inside(get_ROI(last_bbox)) || is_same_armor_by_distance(last_pw, pw)))
        //         {
        //             armor = d;
        //             selected = true;
        //             same_armor = true;
        //             same_id = false;
        //             need_init = false;
        //             break;
        //         }
        //     }
        // }

        // if (!selected && last_track) // 寻找与上次同编号的装甲板
        // {
        //     // handlearmorjump
        //     LOGM_S("[linear] Try ID");
        //     for (auto &d : new_detections)
        //     {
        //         if (d.tag_id == last_bbox.tag_id)
        //         {
        //             LOGM_S("[Linear] Same ID");
        //             armor = d;
        //             selected = true;
        //             same_armor = false;
        //             same_id = true;
        //             need_init = true;
        //             break;
        //         }
        //     }
        // }

        // if (!selected) // 根据离相机中心距离，优先选取近的装甲板
        // {
        //     double min_distance = __DBL_MAX__;
        //     for (auto &d : new_detections)
        //     {
        //         Pos3D m_pw = position_transform.pnp_get_pw(d.pts, d.tag_id);
        //         auto dis = distance_2D(m_pw);
        //         if (dis < min_distance)
        //         {
        //             min_distance = dis;
        //             armor = d;
        //         }
        //     }
        //     LOGM_S("[Linear] Sort by size");
        //     selected = true;
        //     same_armor = false;
        //     same_id = false;
        //     need_init = true;
        // }
        // 不重置ekf
        if (tracker_state != LOST)
        {
            bool selected = false;
            bool same_armor = false;
            bool same_id = false;
            bool need_init = false;
            dt_ = duration_cast<microseconds>(t - last_t).count() / 1e6;
            last_t += duration_cast<microseconds>(t - last_t);
            // KF predict
            Eigen::VectorXd ekf_prediction = ekf.predict();
            // Use KF prediction as default target state if no matched armor is found
            target_state = ekf_prediction;
            if (!new_detections.empty())
            {
                auto predicted_position = getArmorPositionFromState(ekf_prediction);
                for (auto &d : new_detections)
                {
                    auto center = points_center(d.pts);
                    Pos3D pw = position_transform.pnp_get_pw(d.pts, d.tag_id);
                    // 计算ekf预测的位置和实际位置的偏差
                    // 取最小的偏差作为track armor
                    // 若偏差小于允许的最大值max_match_distance_，那么追踪同一块装甲板
                    if (last_track && (center.inside(get_ROI(last_bbox)) || is_same_armor_by_distance(predicted_position, pw)))
                    {
                        armor = d;
                        selected = true;
                        same_armor = true;
                        same_id = true;
                        need_init = false;
                        break;
                    }
                }

                if (!same_armor)
                {
                    // 若最小的偏差大于允许的最大值，那么检测是否有相同id的装甲板
                    // 若有相同id的
                    // Check if there is same id armor in current frame
                    for (auto &d : new_detections)
                    {
                        if (d.tag_id == last_bbox.tag_id)
                        {
                            // Armor jump happens
                            selected = true;
                            same_armor = false;
                            same_id = true;
                            auto p = position_transform.pnp_get_pw(armor.pts, armor.tag_id);
                            handleArmorJump(p, armor, position_transform);
                            break;
                        }
                    }
                }

                if (selected)
                {
                    auto m_pw = position_transform.pnp_get_pw(armor.pts, armor.tag_id);
                    if (same_armor)
                    {
                        // Update EKF
                        double measured_yaw = orientationToYaw(armor, position_transform);
                        Eigen::Vector4d z(m_pw(0), m_pw(1), m_pw(2), measured_yaw);
                        target_state = ekf.update(z);
                    }

                    double ft = FlightTimePredict(m_pw, robot_status.robot_speed_mps);             // ft: 预测弹丸飞行时间
                    auto now_t = std::chrono::steady_clock::now();                        //
                    double process_latency = duration_cast<microseconds>(now_t - t).count() / 1e6; //
                    double t_delay = ft + comm_latency + process_latency;                          //
                    // 根據速度重做
                    /*
                    Pos3D s_pw{p_x(0, 0) + t_delay * p_x(1, 0), p_y(0, 0) + t_delay * p_y(1, 0), m_pw(2, 0)}; // s_pw: ft后预测点
                    */
                    Pos3D s_pw = predict_armor_after_t(target_state, t_delay);
                    Eigen::Vector3d armor_position = getArmorPositionFromState(target_state);
                    Eigen::Vector2d r_vec(armor_position(0), armor_position(1));                                          // 目标装甲板位矢
                    Eigen::Vector2d v_vec = getArmorVectorFromState(target_state);                                         // 目标装甲板速度
                    s_pw(2, 0) -= TrajectoryCompensation(s_pw, robot_status.robot_speed_mps);             // 抬枪后预测点
                    Pos3D s_pc = position_transform.pw_to_pc(s_pw);                                       // point camera: 目标在相机坐标系下的坐标
                    double s_yaw_spd = -(r_vec.dot(v_vec)) / (r_vec.norm() * r_vec.norm()) / M_PI * 180.; // s_yaw_spd: yaw轴速度计算值
                    double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
                    double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

                    send.distance = (float)distance_2D(s_pw);
                    send.yaw_angle = (float)s_yaw;
                    send.yaw_speed = (float)s_yaw_spd;
                    send.pitch_angle = (float)s_pitch;
                }
            }
            // 状态机
            if (tracker_state == DETECTING)
            {
                if (selected)
                {
                    detect_count_++;
                    if (detect_count_ > tracking_threshold_)
                    {
                        detect_count_ = 0;
                        tracker_state = TRACKING;
                    }
                }
                else
                {
                    detect_count_ = 0;
                    tracker_state = LOST;
                }
            }
            else if (tracker_state == TRACKING)
            {
                if (!selected)
                {
                    tracker_state = TEMP_LOST;
                    lost_count_++;
                }
            }
            else if (tracker_state == TEMP_LOST)
            {
                if (!selected)
                {
                    lost_count_++;
                    if (lost_count_ > lost_threshold_)
                    {
                        lost_count_ = 0;
                        tracker_state = LOST;
                    }
                }
                else
                {
                    tracker_state = TRACKING;
                    lost_count_ = 0;
                }
            }
            
        }
        // 重置efk
        else
        {
            // 选取最近的装甲板
            double min_distance = __DBL_MAX__;
            for (auto &d : new_detections)
            {
                Pos3D m_pw = position_transform.pnp_get_pw(d.pts, d.tag_id);
                auto dis = distance_2D(m_pw);
                if (dis < min_distance)
                {
                    min_distance = dis;
                    armor = d;
                }
            }
            LOGM_S("[Linear] Sort by distance");

            Pos3D m_pw = position_transform.pnp_get_pw(armor.pts, armor.tag_id);        // point world: 目标在世界坐标系下的坐标
            initEKF(m_pw, armor, position_transform);                                   // 重置ekf
            double height = TrajectoryCompensation(m_pw, robot_status.robot_speed_mps); // height: 弹道下坠高度
            Pos3D s_pw{m_pw(0, 0), m_pw(1, 0), m_pw(2, 0) - height};                    // 抬枪后预测点
            Pos3D s_pc = position_transform.pw_to_pc(s_pw);                             // point camera: 目标在相机坐标系下的坐标
            double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
            double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

            send.distance = (float)distance_2D(s_pw);
            send.yaw_angle = (float)s_yaw;
            send.yaw_speed = 0.f;
            send.pitch_angle = (float)s_pitch;

            LOGW_S("[Linear] New Filter");

            tracker_state = DETECTING;
        }

        last_track = true;
        last_bbox = armor;
        last_pw = position_transform.pnp_get_pw(armor.pts, armor.tag_id);
        send.traker_state = tracker_state;
    }
}
