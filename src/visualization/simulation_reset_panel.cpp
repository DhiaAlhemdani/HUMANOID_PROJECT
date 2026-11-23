#include <memory>
#include <thread>

#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace legged_robot_mpc_controller
{
class SimulationResetPanel : public rviz_common::Panel
{
public:
  explicit SimulationResetPanel(QWidget *parent = nullptr)
  : rviz_common::Panel(parent),
    node_(std::make_shared<rclcpp::Node>("rviz_simulation_reset_panel"))
  {
    auto *layout = new QVBoxLayout;
    auto *reset_button = new QPushButton("Reset Simulation");
    status_ = new QLabel("Resets MuJoCo and restarts active controllers.");
    reset_button->setToolTip("Restore the MuJoCo model initial state and reinitialize active controllers.");
    layout->addWidget(reset_button);
    layout->addWidget(status_);
    setLayout(layout);

    client_ = node_->create_client<std_srvs::srv::Trigger>(
      "/legged_robot_mpc_controller/reset_simulation");
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });

    connect(reset_button, &QPushButton::clicked, this, [this]() { request_reset(); });
  }

  ~SimulationResetPanel() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(node_);
  }

private:
  void request_reset()
  {
    if (!client_->service_is_ready()) {
      status_->setText("Reset service unavailable. Is MuJoCo running?");
      return;
    }
    status_->setText("Resetting simulation and controllers…");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      const QString result = response->success
        ? QString::fromStdString(response->message)
        : QString::fromStdString("Reset failed: " + response->message);
      QMetaObject::invokeMethod(this, [this, result]() { status_->setText(result); }, Qt::QueuedConnection);
    });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  QLabel *status_{nullptr};
};
}  // namespace legged_robot_mpc_controller

PLUGINLIB_EXPORT_CLASS(legged_robot_mpc_controller::SimulationResetPanel, rviz_common::Panel)
