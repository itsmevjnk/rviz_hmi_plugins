#pragma once

#ifndef Q_MOC_RUN

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rviz_common/panel.hpp"

#include <QProgressBar>
#include <QLabel>

#endif

namespace rviz_hmi_plugins {
    class StatusPanel : public rviz_common::Panel {
        Q_OBJECT

    public:
        explicit StatusPanel(QWidget* parent = 0);

        virtual void load(const rviz_common::Config& config);
        virtual void save(rviz_common::Config config) const;

        void onInitialize() override;
    
    protected:
        void doseCallback(const std_msgs::msg::Float64::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dose_sub_;

        QProgressBar* dose_bar_;
        QLabel* dose_label_;

        const double max_dose_ = 1000.0;
        const double min_dose_ = 0.01;

        const double max_safe_dose_ = 0.57; // maximum safe dose (limit for general public), corresponding to yellow
        // min_dose_ < x < max_safe_dose_ is linearly mapped green -> yellow
        // max_safe_dose_ < x < max_dose_ is logarithmically mapped yellow -> red
 
        const double log_min_dose_ = std::log10(min_dose_);
        const double log_max_dose_ = std::log10(max_dose_);
        const double log_max_safe_dose_ = std::log10(max_safe_dose_);
    };
}