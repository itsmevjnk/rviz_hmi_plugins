#include "teleop_panel.hpp"

#include <stdio.h>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QTimer>
#include <QLabel>

#include <QGamepadKeyNavigation>

#include "pluginlib/class_list_macros.hpp"

namespace rviz_hmi_plugins {
    TeleopPanel::TeleopPanel(QWidget* parent) : rviz_common::Panel(parent) {
        /* list gamepads */
        gamepad_mgr_ = QGamepadManager::instance();
        QList<int> gamepads = gamepad_mgr_->connectedGamepads();
        gamepad_box_ = new QComboBox;
        for (int id : gamepads) {
            QString name = gamepad_mgr_->gamepadName(id);
            gamepad_box_->addItem(name, QVariant(id));
        }
        QGridLayout* gamepad_grid = new QGridLayout;
        gamepad_grid->addWidget(new QLabel("Gamepad:"), 0, 0);
        gamepad_grid->addWidget(gamepad_box_, 0, 1);
        connect(gamepad_mgr_, &QGamepadManager::gamepadConnected, [this](int id) {
            gamepad_box_->addItem(gamepad_mgr_->gamepadName(id), QVariant(id));
        });
        connect(gamepad_mgr_, &QGamepadManager::gamepadDisconnected, [this](int id) {
            int index = gamepad_box_->findData(QVariant(id));
            if (index != -1) gamepad_box_->removeItem(index);
            if (gamepad_id_ == id) deleteGamepad();
        });
        connect(gamepad_box_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int id) {
            updateGamepad();
        });
        updateGamepad(); // update to current

        QGridLayout* button_grid = new QGridLayout;

        forward_button_ = new QPushButton("Forward");
        forward_button_->setShortcut(QKeySequence(Qt::Key_Up));
        connect(forward_button_, &QPushButton::clicked, this, [this]() {
            if (active_button_) active_button_->setStyleSheet("");
            forward_button_->setStyleSheet("background-color: green; color: white;");
            active_button_ = forward_button_;
            state_ = FORWARD;
        });
        button_grid->addWidget(forward_button_, 0, 1);

        backward_button_ = new QPushButton("Backward");
        backward_button_->setShortcut(QKeySequence(Qt::Key_Down));
        connect(backward_button_, &QPushButton::clicked, this, [this]() {
            if (active_button_) active_button_->setStyleSheet("");
            backward_button_->setStyleSheet("background-color: green; color: white;");
            active_button_ = backward_button_;
            state_ = BACKWARD;
        });
        button_grid->addWidget(backward_button_, 2, 1);

        left_button_ = new QPushButton("Turn Left");
        left_button_->setShortcut(QKeySequence(Qt::Key_Left));
        connect(left_button_, &QPushButton::clicked, this, [this]() {
            if (active_button_) active_button_->setStyleSheet("");
            left_button_->setStyleSheet("background-color: green; color: white;");
            active_button_ = left_button_;
            state_ = LEFT;
        });
        button_grid->addWidget(left_button_, 1, 0);

        right_button_ = new QPushButton("Turn Right");
        right_button_->setShortcut(QKeySequence(Qt::Key_Right));
        connect(right_button_, &QPushButton::clicked, this, [this]() {
            if (active_button_) active_button_->setStyleSheet("");
            right_button_->setStyleSheet("background-color: green; color: white;");
            active_button_ = right_button_;
            state_ = RIGHT;
        });
        button_grid->addWidget(right_button_, 1, 2);

        stop_button_ = new QPushButton("STOP");
        stop_button_->setShortcut(QKeySequence(Qt::Key_Space));
        stop_button_->setStyleSheet("background-color: red; color: white;");
        connect(stop_button_, &QPushButton::clicked, this, [this]() {
            if (active_button_) active_button_->setStyleSheet("");
            active_button_ = nullptr;
            state_ = STOP;
        });
        button_grid->addWidget(stop_button_, 1, 1);

        // Add numeric inputs for linear and angular velocity
        QGridLayout* velocity_grid = new QGridLayout;

        QLabel* linear_label = new QLabel("Linear velocity (m/s):");
        linear_spinbox_ = new QDoubleSpinBox;
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
        layout->addLayout(gamepad_grid);
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
        config.mapSetValue("gamepad", gamepad_box_->currentData().toInt());
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

        int id;
        if (config.mapGetInt("gamepad", &id)) {
            int index = gamepad_box_->findData(QVariant(id));
            if (index != -1) { // only proceed if the ID can be found
                updateGamepad(id);
                gamepad_box_->setCurrentIndex(index);
            }
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

    void TeleopPanel::updateGamepad() {
        updateGamepad(gamepad_box_->currentData().toInt());
    }

    void TeleopPanel::updateGamepad(int id) {
        if (gamepad_) deleteGamepad(); // delete current gamepad first (if we're changing gamepads)

        gamepad_ = new QGamepad(id);
        connect(gamepad_, &QGamepad::buttonUpChanged, [this](bool pressed) { gamepadToggleButton(forward_button_, pressed); });
        connect(gamepad_, &QGamepad::buttonDownChanged, [this](bool pressed) { gamepadToggleButton(backward_button_, pressed); });
        connect(gamepad_, &QGamepad::buttonLeftChanged, [this](bool pressed) { gamepadToggleButton(left_button_, pressed); });
        connect(gamepad_, &QGamepad::buttonRightChanged, [this](bool pressed) { gamepadToggleButton(right_button_, pressed); });
        gamepad_id_ = id;
    }

    void TeleopPanel::gamepadToggleButton(QPushButton* button, bool pressed) {
        if (pressed) button->click();
        else if (active_button_ == button) stop_button_->click();
    }

    void TeleopPanel::deleteGamepad() {
        if (!gamepad_) return; // nothing to do
        
        delete gamepad_;
        gamepad_ = nullptr;
        gamepad_id_ = -1;
    }
}

PLUGINLIB_EXPORT_CLASS(rviz_hmi_plugins::TeleopPanel, rviz_common::Panel)
