#ifndef ARM_HARDWARE__ST3215_SYSTEM_HARDWARE_HPP_
#define ARM_HARDWARE__ST3215_SYSTEM_HARDWARE_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/macros.hpp>

#include <memory>
#include <string>
#include <vector>

namespace arm_hardware
{

class St3215SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(St3215SystemHardware)

   hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- 串口 ----
  int fd_ = -1;
  std::string port_;
  int baudrate_ = 1000000;

  // ---- 舵机配置 ----
  std::vector<int> servo_ids_;

  // ---- 硬件缓冲（与控制器共享内存）----
  std::vector<double> hw_commands_;        // 位置命令 (rad)
  std::vector<double> hw_states_pos_;      // 位置状态 (rad)
  std::vector<double> hw_states_vel_;      // 速度状态 (rad/s)
  std::vector<double> hw_states_pos_prev_; // 上一周期位置 (rad)

  // ---- 标定参数（默认中位2048、方向+1，标定后修改）----
  std::vector<double> zero_offsets_;
  std::vector<double> directions_;

  // ---- 协议 ----
  bool openSerial();
  void closeSerial();
  unsigned char checksum(const std::vector<unsigned char> & frame, size_t len = 0) const;
  bool sendFrame(const std::vector<unsigned char> & frame);
  bool readResponse(std::vector<unsigned char> & resp, size_t expected_len, int timeout_ms);
  bool enableTorque(int servo_id, bool on);
  bool writePosition(int servo_id, int position, int speed, int acc);
  int readPosition(int servo_id);   // 返回 0~4095，失败 -1

  void holdCurrentPosition();       // 读当前位置并写入命令/状态缓冲

  double countsToRad(int counts, size_t idx) const;
  int radToCounts(double rad, size_t idx) const;
};

}  // namespace arm_hardware

#endif  // ARM_HARDWARE__ST3215_SYSTEM_HARDWARE_HPP_