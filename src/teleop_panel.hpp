#pragma once

#ifndef Q_MOC_RUN

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rviz_common/panel.hpp"

#include <QPushButton>
#include <QDoubleSpinBox>
#include <QComboBox>

#include <QGamepadManager>
#include <QGamepad>

#endif

namespace rviz_hmi_plugins {
    class TeleopPanel : public rviz_common::Panel {
        Q_OBJECT

    public:
        explicit TeleopPanel(QWidget* parent = 0);

        virtual void load(const rviz_common::Config& config);
        virtual void save(rviz_common::Config config) const;
    
    protected Q_SLOTS:
        void sendVelocity();

        void updateGamepad();
        void updateGamepad(int id);

        void deleteGamepad();

        void gamepadToggleButton(QPushButton* button, bool pressed);
    
    protected:
        QPushButton* active_button_ = nullptr;

        QPushButton* forward_button_;
        QPushButton* backward_button_;
        QPushButton* left_button_;
        QPushButton* right_button_;
        QPushButton* stop_button_;

        QDoubleSpinBox* linear_spinbox_;
        QDoubleSpinBox* angular_spinbox_;
        QComboBox* gamepad_box_;

        QGamepadManager* gamepad_mgr_;
        QGamepad* gamepad_ = nullptr; // current gamepad
        int gamepad_id_ = -1;

        enum {
            STOP,
            FORWARD,
            BACKWARD,
            LEFT,
            RIGHT
        } state_ = STOP;

        double linear_velocity_ = 0.0, angular_velocity_ = 0.0;

        std::shared_ptr<rclcpp::Node> velocity_node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    };
}