
#include <cstdlib>
#include <iostream>
#include <sstream>
#include "servoctl.hpp"

void parsing_error()
{
	std::cerr << "Unable to parse command!" << std::endl;
}

void send_error()
{
	std::cerr << "Unable to send command!" << std::endl;
}

int main(int argc, char **argv)
{
	const char *default_device = "/dev/ttyACM0";

	if (argc >= 2)
	{
		default_device = argv[1];
	}

	std::cout << "Connecting to serial device: " << default_device << std::endl;

	servoctl sc(default_device);

	if (!sc)
	{
		std::cerr << "Unable to open serial device: " << default_device << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Command syntax: <cmd> [<params>...]\n";
	std::cout << "Valid commands are:\n";
	std::cout << " s  <servo_index> <angle>                  Set servo angle.\n";
	std::cout << " st <servo_index> <angle> <timing_in_ms>   Set servo angle with timing.\n";
	std::cout << " sa <angle>                                Set servo angle.\n";
	std::cout << " sat <angle> <timing_in_ms>                Set servo angle with timing.\n";
	std::cout << " es <servo_index>                          Enable servos.\n";
	std::cout << " ds <servo_index>                          Disable servo.\n";
	std::cout << " ea                                        Enable all servos.\n";
	std::cout << " da                                        Disable all servos.\n";
	std::cout << " sc <si> <min_a> <max_a> <min_t_ms> <min_t_ms> \n";
	std::cout << " gc                                        Get all calibrations\n";
	std::cout << " ts <servo_index> <reps> <timing_in_ms>    Do a simple test on a servo channel (window wiper).\n";
	std::cout << " tsa <reps> <timing_in_ms>                 Do a simple test on all servo channels (window wiper).\n";
	std::cout << " test1                                     Test sequence 1.\n";
	std::cout << " test2                                     Test sequence 2.\n";
	std::cout << " test3                                     Test sequence 3.\n";
	std::cout << " q                                         Exit test console.\n";

	std::string line, cmd;
	int servo_index, angle, timing, reps;
	uint8_t min_angles[NUM_SERVOS], max_angles[NUM_SERVOS];
	uint16_t min_len_us[NUM_SERVOS], max_len_us[NUM_SERVOS];
	int min_angle, max_angle, min_l_us, max_l_us;
	int enabled;

	if (!sc.enable_all_servos(true))
	{
		send_error();
	}

	for (;;)
	{
		std::cout << ">>> ";
		std::getline(std::cin, line);
		std::stringstream ss(line);
		if (!(ss >> cmd))
		{
			continue;
		}
		if (cmd == "s")
		{
			if (!(ss >> servo_index >> angle))
			{
				parsing_error();
				continue;
			}
			if (!sc.set_angle(servo_index, angle))
			{
				send_error();
			}
		}
		else if (cmd == "st")
		{
			if (!(ss >> servo_index >> angle >> timing))
			{
				parsing_error();
				continue;
			}
			if (!sc.set_angle_timed(servo_index, angle, timing))
			{
				send_error();
			}
		}
		else if (cmd == "sa")
		{
			if (!(ss >> angle))
			{
				parsing_error();
				continue;
			}
			std::uint8_t angles[24];
			std::fill_n(angles, 24, angle);
			if (!sc.set_all_angles(angles))
			{
				send_error();
			}
		}
		else if (cmd == "sat")
		{
			if (!(ss >> angle >> timing))
			{
				parsing_error();
				continue;
			}
			std::uint8_t angles[NUM_SERVOS];
			std::uint16_t timings[NUM_SERVOS];
			std::fill_n(angles, NUM_SERVOS, angle);
			std::fill_n(timings, NUM_SERVOS, timing);
			if (!sc.set_all_angles_timed(angles, timings))
			{
				send_error();
			}
		}
		else if (cmd == "es")
		{
			if (!(ss >> servo_index))
			{
				parsing_error();
				continue;
			}
			if (!sc.enable_servo(servo_index, true))
			{
				send_error();
			}
		}
		else if (cmd == "ds")
		{
			if (!(ss >> servo_index))
			{
				parsing_error();
				continue;
			}
			if (!sc.enable_servo(servo_index, false))
			{
				send_error();
			}
		}
		else if (cmd == "ea")
		{
			if (!sc.enable_all_servos(true))
			{
				send_error();
			}
		}
		else if (cmd == "da")
		{
			if (!sc.enable_all_servos(false))
			{
				send_error();
			}
		}
		else if (cmd == "l")
		{
			if (!(ss >> enabled))
			{
				parsing_error();
				continue;
			}
			if (!sc.set_led(static_cast<led_state>(enabled)))
			{
				send_error();
			}
		}
		else if (cmd == "ts")
		{
			if (!(ss >> servo_index >> reps >> timing))
			{
				parsing_error();
				continue;
			}
			for (int r = 0; r < reps; r++)
			{
				if (!sc.set_angle_timed(servo_index, 0, timing))
				{
					send_error();
				}
				if (!sc.set_angle_timed(servo_index, 180, timing))
				{
					send_error();
				}
			}
		}
		else if (cmd == "sc") 
		{
			if (!(ss >> servo_index >> min_angle >> max_angle >> min_l_us >> max_l_us))
			{
				parsing_error();
				continue;
			}
			if(!sc.set_calibration(servo_index, min_angle, max_angle, min_l_us, max_l_us)) {
				send_error();
			}			
		}
		else if (cmd == "gc") 
		{
			if(!sc.get_calibrations(min_angles, max_angles, min_len_us, max_len_us)) {
				send_error();
				continue;
			}
			for(int i = 0; i < NUM_SERVOS; i++) {
				std::cout << i << ": " << (int)min_angles[i] << "° - " << (int)max_angles[i] << "°, "
					<< min_len_us[i] << " µs - " << max_len_us[i] << " µs" << std::endl;
			}
		}
		else if (cmd == "tsa")
		{
			if (!(ss >> reps >> timing))
			{
				parsing_error();
				continue;
			}
			std::uint8_t angles0[24];
			std::uint8_t angles180[24];
			std::uint16_t timings[24];
			std::fill_n(angles0, 24, 0);
			std::fill_n(angles180, 24, 180);
			std::fill_n(timings, 24, timing);
			for (int r = 0; r < reps; r++)
			{
				if (!sc.set_all_angles_timed(angles0, timings))
				{
					send_error();
				}
				if (!sc.set_all_angles_timed(angles180, timings))
				{
					send_error();
				}
			}
		}
		else if (cmd == "test1")
		{
			for (int a = 0; a <= 180; a += 5)
			{
				for (int i = 0; i < 3; i++)
				{
					if (!sc.set_angle(i, a))
					{
						send_error();
					}
				}
			}
			for (int a = 180; a >= 0; a -= 5)
			{
				for (int i = 0; i < 3; i++)
				{
					if (!sc.set_angle(i, a))
					{
						send_error();
					}
				}
			}
		}
		else if (cmd == "test2")
		{
			int a = 0;
			for (int i = 0; i < 3; i++)
			{
				for (int t = 1000; t <= 6000; t += 1000)
				{
					if (!sc.set_angle_timed(i, a, t))
					{
						send_error();
					}
					if (a == 0)
						a = 180;
					else if (a == 180)
						a = 0;
				}
			}
		}
		else if (cmd == "test3")
		{
			sc.set_angle(0, 0);
			sc.set_angle(1, 0);
			sc.set_angle(2, 0);
			for (int i = 0; i < 5; i++)
			{
				sc.set_angle_timed(0, 180, 1000);
				sc.set_angle_timed(1, 180, 3000);
				sc.set_angle_timed(2, 180, 6000);
				sc.flush();
				sc.set_angle_timed(0, 0, 1000);
				sc.set_angle_timed(1, 0, 3000);
				sc.set_angle_timed(2, 0, 6000);
				sc.flush();
			}
		}
		else if (cmd == "q")
		{
			break;
		}
		else
		{
			parsing_error();
		}
	}

	return EXIT_SUCCESS;
}
