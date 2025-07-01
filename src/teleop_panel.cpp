#include "teleop_panel.hpp"

#include <stdio.h>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QTimer>
#include <QLabel>
#include <QDoubleSpinBox>

#include "pluginlib/class_list_macros.hpp"

namespace rviz_hmi_plugins {
    TeleopPanel::TeleopPanel(QWidget* parent) : rviz_common::Panel(parent) {
        QGridLayout* button_grid = new QGridLayout;

        QPushButton* forward_button = new QPushButton("Forward");
        forward_button->setShortcut(QKeySequence(Qt::Key_Up));
        connect(forward_button, &QPushButton::clicked, this, [forward_button, this]() {
            if (active_button_) active_button_->setStyleSheet("");
            forward_button->setStyleSheet("background-color: green; color: white;");
            active_button_ = forward_button;
            state_ = FORWARD;
        });
        button_grid->addWidget(forward_button, 0, 1);

        QPushButton* backward_button = new QPushButton("Backward");
        backward_button->setShortcut(QKeySequence(Qt::Key_Down));
        connect(backward_button, &QPushButton::clicked, this, [backward_button, this]() {
            if (active_button_) active_button_->setStyleSheet("");
            backward_button->setStyleSheet("background-color: green; color: white;");
            active_button_ = backward_button;
            state_ = BACKWARD;
        });
        button_grid->addWidget(backward_button, 2, 1);

        QPushButton* left_button = new QPushButton("Turn Left");
        left_button->setShortcut(QKeySequence(Qt::Key_Left));
        connect(left_button, &QPushButton::clicked, this, [left_button, this]() {
            if (active_button_) active_button_->setStyleSheet("");
            left_button->setStyleSheet("background-color: green; color: white;");
            active_button_ = left_button;
            state_ = LEFT;
        });
        button_grid->addWidget(left_button, 1, 0);

        QPushButton* right_button = new QPushButton("Turn Right");
        right_button->setShortcut(QKeySequence(Qt::Key_Right));
        connect(right_button, &QPushButton::clicked, this, [right_button, this]() {
            if (active_button_) active_button_->setStyleSheet("");
            right_button->setStyleSheet("background-color: green; color: white;");
            active_button_ = right_button;
            state_ = RIGHT;
        });
        button_grid->addWidget(right_button, 1, 2);

        QPushButton* stop_button = new QPushButton("STOP");
        stop_button->setShortcut(QKeySequence(Qt::Key_Space));
        stop_button->setStyleSheet("background-color: red; color: white;");
        connect(stop_button, &QPushButton::clicked, this, [stop_button, this]() {
            if (active_button_) active_button_->setStyleSheet("");
            active_button_ = nullptr;
            state_ = STOP;
        });
        button_grid->addWidget(stop_button, 1, 1);

        // Add numeric inputs for linear and angular velocity
        QGridLayout* velocity_grid = new QGridLayout;

        QLabel* linear_label = new QLabel("Linear velocity (m/s):");
        QDoubleSpinBox* linear_spinbox_ = new QDoubleSpinBox;
        linear_spinbox_->setRange(0.0, 5.0);
        linear_spinbox_->setSingleStep(0.1);
        linear_spinbox_->setValue(0.5);        
        connect(linear_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double value) { linear_velocity_ = value; });
        velocity_grid->addWidget(linear_label, 0, 0);
        velocity_grid->addWidget(linear_spinbox_, 0, 1);

        QLabel* angular_label = new QLabel("Angular velocity (rad/s):");
        angular_spinbox_ = new QDoubleSpinBox;
        angular_spinbox_->setRange(0.0, 31.4);
        angular_spinbox_->setSingleStep(0.1);
        angular_spinbox_->setValue(1.0);
        connect(angular_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double value) { angular_velocity_ = value; });
        velocity_grid->addWidget(angular_label, 1, 0);
        velocity_grid->addWidget(angular_spinbox_, 1, 1);
        linear_velocity_ = linear_spinbox_->value();
        angular_velocity_ = angular_spinbox_->value();
        
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addLayout(button_grid);
        layout->addLayout(velocity_grid);
        setLayout(layout);

        QTimer* pub_timer = new QTimer(this);
        connect(pub_timer, SIGNAL(timeout()), this, SLOT(sendVelocity()));
        pub_timer->start(100);

        velocity_node_ = std::make_shared<rclcpp::Node>("teleop_panel_velocity_node");
        velocity_pub_ = velocity_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    }

    void TeleopPanel::save(rviz_common::Config config) const {
        rviz_common::Panel::save(config);
config.mapSetValue("linear_vel", linear_velocity_);
        config.mapSetValue("angular_vel", angular_velocity_);
    }

    void TeleopPanel::load(const rviz_common::Config& config) {
        rviz_common::Panel::load(config);
float val;
        if (config.mapGetFloat("linear_vel", &val)) {
            linear_velocity_ = val;
            linear_spinbox_->setValue(val);
        }
        if (config.mapGetFloat("angular_vel", &val)) {
            angular_velocity_ = val;
            angular_spinbox_->setValue(val);
        }
    }

    void TeleopPanel::sendVelocity() {
        if (rclcpp::ok() && velocity_pub_ != NULL) {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = msg.linear.y = msg.linear.z = 0;
            msg.angular.x = msg.angular.y = msg.angular.z = 0;
            switch (state_) {
                case FORWARD: msg.linear.x = linear_velocity_; break;
                case BACKWARD: msg.linear.x = -linear_velocity_; break;
                case LEFT: msg.angular.z = angular_velocity_; break;
                case RIGHT: msg.angular.z = -angular_velocity_; break;
                default: break;
            }
            velocity_pub_->publish(msg);
        }
    }
}

PLUGINLIB_EXPORT_CLASS(rviz_hmi_plugins::TeleopPanel, rviz_common::Panel)
