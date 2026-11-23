#include <memory>

#include <QPushButton>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace legged_robot_mpc_controller::visualization
{
class SimulationResetPanel final : public rviz_common::Panel
{
public:
  explicit SimulationResetPanel(QWidget *parent = nullptr) : rviz_common::Panel(parent)
  {
    auto *layout = new QVBoxLayout(this);
    auto *button = new QPushButton("Reset Simulation", this);
    layout->addWidget(button);
    QObject::connect(button, &QPushButton::clicked, this, [this]() { request_reset(); });
  }

  void onInitialize() override
  {
    const auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    reset_client_ = node->create_client<std_srvs::srv::Trigger>(
      "/legged_robot_mpc_controller/reset_simulation");
  }

private:
  void request_reset()
  {
    if (!reset_client_ || !reset_client_->service_is_ready()) {
      RCLCPP_WARN(rclcpp::get_logger("simulation_reset_panel"), "Coordinated reset service is not ready");
      return;
    }
    reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
};
}  // namespace legged_robot_mpc_controller::visualization

PLUGINLIB_EXPORT_CLASS(
  legged_robot_mpc_controller::visualization::SimulationResetPanel,
  rviz_common::Panel)
