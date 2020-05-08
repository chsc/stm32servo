
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <thread>
#include <chrono>
#include <iostream>
#include "servoctl.hpp"

// https://en.wikibooks.org/wiki/Serial_Programming/termios

constexpr std::uint8_t SERVO_CMD_SET_SERVO_TIMED = 0x01;
constexpr std::uint8_t SERVO_CMD_SET_ALL_SERVOS_TIMED = 0x11;
constexpr std::uint8_t SERVO_CMD_SET_SERVO = 0x02;
constexpr std::uint8_t SERVO_CMD_SET_ALL_SERVOS = 0x12;
constexpr std::uint8_t SERVO_CMD_ENABLE_SERVO = 0x03;
constexpr std::uint8_t SERVO_CMD_ENABLE_ALL_SERVOS = 0x13;
constexpr std::uint8_t SERVO_CMD_SET_CALIBRATION = 0x04;
constexpr std::uint8_t SERVO_CMD_GET_CALIBRATIONS = 0x15;
constexpr std::uint8_t SERVO_CMD_SET_LED = 0xa0;

servoctl::servoctl(const std::string &device) : fd(-1)
{
	open(device);
}

servoctl::servoctl(servoctl &&sc) : fd(sc.fd)
{
	sc.fd = -1;
}

servoctl::~servoctl()
{
	close();
}

servoctl::operator bool() const
{
	return fd != -1;
}

servoctl &servoctl::operator=(servoctl &&sc)
{
	fd = sc.fd;
	sc.fd = -1;
	return *this;
}

void servoctl::open(const std::string &device)
{
	struct termios attribs;

	fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		std::cerr << "servoctl: " << strerror(errno) << std::endl;
		return;
	}

	// 8 bit, no parity, 1 stop bit, no hw flow control
	attribs.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	attribs.c_iflag = IGNPAR;
	attribs.c_oflag = 0;
	attribs.c_lflag = 0; // non canonical
	attribs.c_cc[VTIME] = 1; // 0.1 sec timeout
	attribs.c_cc[VMIN] = 1; // blocking read until 1 character

	if (::tcsetattr(fd, TCSANOW, &attribs) < 0)
	{
		std::cerr << "servoctl: " << strerror(errno) << std::endl;
		close();
		return;
	}

	tcflush(fd, TCIOFLUSH);
}

void servoctl::close()
{
	if (fd != -1)
	{
		::close(fd);
		fd = -1;
	}
}

std::size_t servoctl::write(const std::uint8_t *data, std::size_t len)
{
	return ::write(fd, data, len);
}

std::size_t servoctl::read(std::uint8_t *data, std::size_t len)
{
	size_t rd = 0;
	while(rd < len) {
		int r = ::read(fd, data + rd, len - rd);
		if(r == -1) {
			return -1;
		}
		rd += r;
	}
	return rd;
}

bool servoctl::flush_serial()
{
	return ::tcflush(fd, TCIOFLUSH) == 0;
}

bool servoctl::set_led(led_state state)
{
	std::uint8_t data[] = {
		SERVO_CMD_SET_LED,
		static_cast<std::uint8_t>(state)};
	return write(data, 2) == 2;
}

bool servoctl::set_angle_timed(std::uint8_t servo_index, std::uint8_t angle, std::uint16_t time)
{
	std::uint8_t data[] = {
		SERVO_CMD_SET_SERVO_TIMED,
		servo_index,
		angle,
		(uint8_t)(time & 0xff),
		(uint8_t)(time >> 8)};
	return write(data, 5) == 5;
}

bool servoctl::set_all_angles_timed(std::uint8_t *angles, std::uint16_t *times_ms)
{
	std::uint8_t cmd = SERVO_CMD_SET_ALL_SERVOS_TIMED;
	if(write(&cmd, 1) != 1) return false;
	if(write(angles, NUM_SERVOS) != NUM_SERVOS) return false;
	return write(reinterpret_cast<std::uint8_t *>(times_ms), 24 * sizeof(std::uint16_t)) == 24 * sizeof(std::uint16_t);
}

bool servoctl::set_angle(std::uint8_t servo_index, std::uint8_t angle)
{
	std::uint8_t data[] = {SERVO_CMD_SET_SERVO, servo_index, angle};
	return write(data, 3) == 3;
}

bool servoctl::set_all_angles(uint8_t *angles)
{
	std::uint8_t cmd = SERVO_CMD_SET_ALL_SERVOS;
	if (write(&cmd, 1) != 1)
		return false;
	return write(angles, NUM_SERVOS) == NUM_SERVOS;
}

bool servoctl::set_calibration(
	std::uint8_t servo_index,
	std::uint8_t min_angle, std::uint8_t max_angle,
	std::uint16_t min_phase_us, std::uint16_t max_phase_us)
{
	std::uint8_t data[] = {
		SERVO_CMD_SET_CALIBRATION,
		servo_index,
		min_angle, max_angle,
		(uint8_t)(min_phase_us & 0xff), (uint8_t)((min_phase_us >> 8) & 0xff),
		(uint8_t)(max_phase_us & 0xff), (uint8_t)((max_phase_us >> 8) & 0xff)};

	return write(data, 8) == 8;
}

bool servoctl::get_calibrations(
    std::uint8_t min_angles[], std::uint8_t max_angles[],
    std::uint16_t min_len_us[], std::uint16_t max_len_us[])
{
	std::uint8_t cmd = SERVO_CMD_GET_CALIBRATIONS;
	if(write(&cmd, 1) != 1) return false;

	if(read(min_angles, NUM_SERVOS) != NUM_SERVOS) {
		std::cout << "x" << std::endl;
		return false;
	}
	if(read(max_angles, NUM_SERVOS) != NUM_SERVOS) {
		std::cout << "xx" << std::endl;
		return false;
	}
	if(read(reinterpret_cast<std::uint8_t*>(min_len_us), sizeof(uint16_t[NUM_SERVOS])) != sizeof(uint16_t[NUM_SERVOS]))  {
		std::cout << "xxx" << std::endl;
		return false;
	}
	if(read(reinterpret_cast<std::uint8_t*>(max_len_us), sizeof(uint16_t[NUM_SERVOS])) != sizeof(uint16_t[NUM_SERVOS])) {
		std::cout << "xxxx" << std::endl;
		return false;
	}
	return true;
}

bool servoctl::enable_servo(uint8_t servo_index, bool enable)
{
	std::uint8_t data[] = {
		SERVO_CMD_ENABLE_SERVO,
		servo_index,
		enable};
	return write(data, 3) == 3;
}

bool servoctl::enable_all_servos(bool enable)
{
	std::uint8_t data[] = {SERVO_CMD_ENABLE_ALL_SERVOS, enable ? (uint8_t)0x01 : (uint8_t)0x00};
	return write(data, 2) == 2;
}

bool servoctl::flush()
{
	return flush_serial();
}
