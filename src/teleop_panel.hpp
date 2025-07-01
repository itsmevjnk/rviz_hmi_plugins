#pragma once

#ifndef Q_MOC_RUN

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rviz_common/panel.hpp"

#include <QPushButton>
#include <QDoubleSpinBox>

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
    
    protected:
        QPushButton* active_button_ = nullptr;

        QDoubleSpinBox* linear_spinbox_;
        QDoubleSpinBox* angular_spinbox_;

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