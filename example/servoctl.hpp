
#ifndef _SERVOCTL_HPP_INCLUDED_
#define _SERVOCTL_HPP_INCLUDED_

#include <cstdint>
#include <string>

constexpr std::uint8_t NUM_SERVOS = 24;

enum class led_state : std::uint8_t
{
  off = 0,
  on = 1,
  toggle = 2,
  automatic = 3
};

class servoctl
{
public:
  servoctl(const servoctl &sc) = delete;
  servoctl(servoctl &&sc);
  servoctl(const std::string &device);
  ~servoctl();

  operator bool() const;
  servoctl &operator=(servoctl &&sc);

  bool set_led(led_state);

  bool enable_servo(uint8_t servo_index, bool enable);
  bool enable_all_servos(bool enable);

  bool set_angle_timed(std::uint8_t servo_index, std::uint8_t angle, std::uint16_t time_ms);
  bool set_all_angles_timed(std::uint8_t *angle, std::uint16_t *time_ms);
  bool set_angle(uint8_t servo_index, uint8_t angle);
  bool set_all_angles(uint8_t *angles);

  bool set_calibration(
    std::uint8_t servo_index,
    std::uint8_t min_angle, std::uint8_t max_angle,
    std::uint16_t min_phase_us, std::uint16_t max_phase_us);

  bool get_calibrations(
    std::uint8_t min_angle[], std::uint8_t max_angle[],
    std::uint16_t min_phase_us[], std::uint16_t max_phase_us[]);

  bool flush();

private:
  void open(const std::string &device);
  void close();

  std::size_t write(const std::uint8_t *data, std::size_t len);
  std::size_t read(std::uint8_t *data, std::size_t len);

  bool flush_serial();

  int fd;
};

#endif
