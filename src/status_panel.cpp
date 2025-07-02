#include "status_panel.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <stdio.h>
#include <QGridLayout>
#include <QProgressBar>

#include <rviz_common/display_context.hpp>

namespace rviz_hmi_plugins {
    StatusPanel::StatusPanel(QWidget* parent) : rviz_common::Panel(parent) {
        QGridLayout* layout = new QGridLayout;
        layout->setColumnStretch(0, 0); // do not stretch left column
        layout->setColumnStretch(2, 0);

        dose_bar_ = new QProgressBar;
        dose_label_ = new QLabel("N/A");
        dose_bar_->setTextVisible(false);
        layout->addWidget(new QLabel("Radiation (μSv/h):"), 0, 0);
        layout->addWidget(dose_bar_, 0, 1);
        layout->addWidget(dose_label_, 0, 2);
        setLayout(layout);
    }

    void StatusPanel::doseCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        double dose = msg->data;
        // RCLCPP_INFO(status_node_->get_logger(), "dose: %lf uSv/h", dose);
        
        double value = std::clamp(
            (std::log10(dose) - log_min_dose_) / (log_max_dose_ - log_min_dose_),
            0.0, 1.0
        ); // 0.0 -> 1.0
        dose_bar_->setValue(static_cast<int>(value * 100.0));
        dose_label_->setText(QString("%1").arg(dose, 0, 'f', (dose < 10) ? 2 : 1));
        
        QColor color;
        if (dose < max_safe_dose_)
            color.setRgbF(0.0 + std::clamp(dose / max_safe_dose_, 0.0, 1.0), 1.0, 0.0);
        else
            color.setRgbF(1.0, 1.0 - std::clamp((std::log10(dose) - log_max_safe_dose_) / (log_max_dose_ - log_max_safe_dose_), 0.0, 1.0), 0.0);
        
        QString bar_style = QString(R"(
            QProgressBar::chunk {
                background-color: %1;
            })").arg(color.name());
        dose_bar_->setStyleSheet(bar_style);
    }

    void StatusPanel::onInitialize() {
        auto ros_node_abstraction_weak = this->getDisplayContext()->getRosNodeAbstraction();
        auto ros_node_abstraction = ros_node_abstraction_weak.lock();
        if (ros_node_abstraction) {
            auto rviz_ros_node = ros_node_abstraction->get_raw_node();
            dose_sub_ = rviz_ros_node->create_subscription<std_msgs::msg::Float64>(
                "/dose", rclcpp::SensorDataQoS(),
                std::bind(&StatusPanel::doseCallback, this, std::placeholders::_1)
            );
        }
    }

    void StatusPanel::save(rviz_common::Config config) const {
        rviz_common::Panel::save(config);
    }

    void StatusPanel::load(const rviz_common::Config& config) {
        rviz_common::Panel::load(config);
    }
}

PLUGINLIB_EXPORT_CLASS(rviz_hmi_plugins::StatusPanel, rviz_common::Panel)

