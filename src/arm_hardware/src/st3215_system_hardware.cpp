#include "arm_hardware/st3215_system_hardware.hpp"

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>

namespace arm_hardware
{

// ---- ST3215 寄存器地址（与 scservo_sdk 一致）----
namespace reg
{
constexpr int TORQUE_ENABLE = 40;    // 0x28
constexpr int ACC           = 41;    // 0x29
constexpr int PRESENT_POS_L = 56;    // 0x38
}  // namespace reg

// ---- 指令码 ----
namespace inst
{
constexpr unsigned char READ  = 0x02;
constexpr unsigned char WRITE = 0x03;
}  // namespace inst

constexpr double TWO_PI = 6.283185307179586;
constexpr double COUNTS_PER_REV = 4096.0;

// ============================================================
// on_init: 读参数、校验接口定义
// ============================================================
hardware_interface::CallbackReturn St3215SystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_ = info_.hardware_parameters.at("port");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

  // 解析 servo_ids: "1,2,3,4,5,6"
  const std::string ids = info_.hardware_parameters.at("servo_ids");
  size_t start = 0;
  while (start < ids.size()) {
    size_t comma = ids.find(',', start);
    if (comma == std::string::npos) comma = ids.size();
    servo_ids_.push_back(std::stoi(ids.substr(start, comma - start)));
    start = comma + 1;
  }

  // 校验每个关节的接口
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(rclcpp::get_logger("St3215SystemHardware"),
                   "关节 %s 需要 1 个 position 命令接口", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    bool has_pos = false, has_vel = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_pos = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY) has_vel = true;
    }
    if (!has_pos || !has_vel) {
      RCLCPP_ERROR(rclcpp::get_logger("St3215SystemHardware"),
                   "关节 %s 需要 position + velocity 状态接口", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 分配缓冲
  const size_t n = info_.joints.size();
  hw_commands_.resize(n);
  hw_states_pos_.resize(n);
  hw_states_vel_.resize(n);
  hw_states_pos_prev_.resize(n);
  // 来自 Seeed_RoboController/json/my_awesome_follower_arm.json 的真机标定。
  // 关节顺序与 <ros2_control> 一致：
  // shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
  const std::vector<double> zero_offsets = {
    2050.0, 2070.0, 2036.0, 2026.0, 1017.0, 2500.0};
  const std::vector<double> directions = {
    1.0, 1.0, 1.0, 1.0, -1.0, 1.0};

  if (n != zero_offsets.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("St3215SystemHardware"),
                 "关节数量不匹配: URDF=%zu, 标定=%zu",
                 n, zero_offsets.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  zero_offsets_ = zero_offsets;
  directions_ = directions;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// export: 把内部缓冲暴露给 ros2_control
// ============================================================
std::vector<hardware_interface::StateInterface>
St3215SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_pos_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
St3215SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

// ============================================================
// 生命周期
// ============================================================
hardware_interface::CallbackReturn St3215SystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!openSerial()) {
    RCLCPP_ERROR(rclcpp::get_logger("St3215SystemHardware"), "串口打开失败");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 上电即保持力矩，MoveIt（RViz 或 GUI）随时可以执行轨迹
  for (int id : servo_ids_) {
    if (!enableTorque(id, true)) {
      RCLCPP_WARN(rclcpp::get_logger("St3215SystemHardware"),
                  "力矩使能失败 ID=%d", id);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn St3215SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  // 用当前位置初始化命令，避免控制器接管瞬间跳到 0 弧度
  holdCurrentPosition();
  for (int id : servo_ids_) {
    if (!enableTorque(id, true)) {
      RCLCPP_WARN(rclcpp::get_logger("St3215SystemHardware"),
                  "力矩使能失败 ID=%d", id);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn St3215SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (int id : servo_ids_) {
    enableTorque(id, false);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn St3215SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn St3215SystemHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// read: 从舵机读位置 → 弧度 → 状态缓冲
// ============================================================
hardware_interface::return_type St3215SystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    int counts = readPosition(servo_ids_[i]);
    if (counts >= 0) {
      double rad = countsToRad(counts, i);
      hw_states_pos_[i] = rad;
      if (dt > 1e-6) {
        hw_states_vel_[i] = (rad - hw_states_pos_prev_[i]) / dt;
      }
      hw_states_pos_prev_[i] = rad;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  return hardware_interface::return_type::OK;
}

// ============================================================
// write: 命令缓冲 → 弧度 → 舵机位置值 → 发串口
// ============================================================
hardware_interface::return_type St3215SystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    int counts = radToCounts(hw_commands_[i], i);
    // speed 2400 steps/s ≈ 3.7 rad/s，acc 100 ≈ 15 rad/s²：
    // 给轨迹跟踪留余量，避免舵机自身限速成为瓶颈
    writePosition(servo_ids_[i], counts, 2400, 100);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  return hardware_interface::return_type::OK;
}

// ============================================================
// 位置保持
// ============================================================
void St3215SystemHardware::holdCurrentPosition()
{
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    int counts = readPosition(servo_ids_[i]);
    if (counts >= 0) {
      const double rad = countsToRad(counts, i);
      hw_commands_[i] = rad;
      hw_states_pos_[i] = rad;
      hw_states_pos_prev_[i] = rad;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("St3215SystemHardware"),
                  "读取当前位置失败 ID=%d", servo_ids_[i]);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
}

// ============================================================
// 坐标换算
// ============================================================
double St3215SystemHardware::countsToRad(int counts, size_t idx) const
{
  double rad = (counts - zero_offsets_[idx]) * TWO_PI / COUNTS_PER_REV;
  return directions_[idx] * rad;
}

int St3215SystemHardware::radToCounts(double rad, size_t idx) const
{
  double counts = zero_offsets_[idx] + (directions_[idx] * rad) * COUNTS_PER_REV / TWO_PI;
  if (counts < 0.0) counts = 0.0;
  if (counts > 4095.0) counts = 4095.0;
  return static_cast<int>(std::lround(counts));
}

// ============================================================
// 串口
// ============================================================
bool St3215SystemHardware::openSerial()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("St3215SystemHardware"),
                 "无法打开串口 %s", port_.c_str());
    return false;
  }

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;   // 100ms 读超时

  speed_t baud;
  switch (baudrate_) {
    case 1000000: baud = B1000000; break;
    case 500000:  baud = B500000;  break;
    case 115200:  baud = B115200;  break;
    default:      baud = B1000000; break;
  }
  cfsetispeed(&tio, baud);
  cfsetospeed(&tio, baud);
  tcsetattr(fd_, TCSANOW, &tio);
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void St3215SystemHardware::closeSerial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// ============================================================
// SCS 协议
// ============================================================
unsigned char St3215SystemHardware::checksum(
  const std::vector<unsigned char> & frame, size_t len) const
{
  if (len == 0) len = frame.size();
  unsigned int sum = 0;
  for (size_t i = 2; i < len; ++i) sum += frame[i];   // 不含头、不含校验位
  return static_cast<unsigned char>(~sum & 0xFF);
}

bool St3215SystemHardware::sendFrame(const std::vector<unsigned char> & frame)
{
  tcflush(fd_, TCIFLUSH);   // 清掉残留应答
  ssize_t n = ::write(fd_, frame.data(), frame.size());
  if (n != static_cast<ssize_t>(frame.size())) return false;
  tcdrain(fd_);             // 等发送完（半双工切方向）
  return true;
}

bool St3215SystemHardware::readResponse(
  std::vector<unsigned char> & resp, size_t expected_len, int timeout_ms)
{
  resp.clear();
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (resp.size() < expected_len && std::chrono::steady_clock::now() < deadline) {
    unsigned char buf[16];
    ssize_t n = ::read(fd_, buf, expected_len - resp.size());
    if (n > 0) {
      resp.insert(resp.end(), buf, buf + n);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return resp.size() == expected_len;
}

bool St3215SystemHardware::enableTorque(int servo_id, bool on)
{
  // WRITE: FF FF ID 04 03 28 data CHK
   std::vector<unsigned char> tx = {0xFF, 0xFF, static_cast<unsigned char>(servo_id),
                                   0x04, inst::WRITE, reg::TORQUE_ENABLE,
                                   static_cast<unsigned char>(on ? 1 : 0), 0u};
  tx.back() = checksum(tx);
  if (!sendFrame(tx)) return false;
  std::vector<unsigned char> rx;
  return readResponse(rx, 6, 50);
}

bool St3215SystemHardware::writePosition(int servo_id, int position, int speed, int acc)
{
  // 复刻 WritePosEx: 从地址 ACC(41) 写 7 字节 [acc,posL,posH,0,0,spdL,spdH]
  // LEN = inst + addr + 7 + chk = 10
  std::vector<unsigned char> tx = {0xFF, 0xFF, static_cast<unsigned char>(servo_id),
                                   0x0A, inst::WRITE, reg::ACC, static_cast<unsigned char>(acc),
                                   static_cast<unsigned char>(position & 0xFF),
                                   static_cast<unsigned char>((position >> 8) & 0xFF),
                                   0x00, 0x00,   // GOAL_TIME = 0
                                   static_cast<unsigned char>(speed & 0xFF),
                                   static_cast<unsigned char>((speed >> 8) & 0xFF),
                                   0x00};
  tx.back() = checksum(tx);
  if (!sendFrame(tx)) return false;
  std::vector<unsigned char> rx;
  return readResponse(rx, 6, 50);
}

int St3215SystemHardware::readPosition(int servo_id)
{
  // READ: FF FF ID 04 02 38 02 CHK
  std::vector<unsigned char> tx = {0xFF, 0xFF, static_cast<unsigned char>(servo_id),
                                   0x04, inst::READ, reg::PRESENT_POS_L, 0x02, 0x00};
  tx.back() = checksum(tx);
  if (!sendFrame(tx)) return -1;

  // 应答 8 字节: FF FF ID 04 ERR POS_L POS_H CHK
  std::vector<unsigned char> rx;
  if (!readResponse(rx, 8, 50)) return -1;
  if (rx.size() < 8 || rx[0] != 0xFF || rx[1] != 0xFF ||
      rx[2] != static_cast<unsigned char>(servo_id)) return -1;
  if (rx[7] != checksum(rx, rx.size() - 1)) return -1;
  return rx[5] | (rx[6] << 8);
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(arm_hardware::St3215SystemHardware, hardware_interface::SystemInterface)