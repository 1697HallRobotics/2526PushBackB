#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "recording.h"
#include "tenna_gif.h"

//#define RECORDING

static lv_obj_t* gif_img;
int gifTimer = 0;
bool tru = false;

void update_gif(lv_timer_t* timer) {
	lv_image_set_src(gif_img, &(tenna_gif[(gifTimer) % 7]));
	gifTimer++;
	lv_obj_invalidate(gif_img);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// The primary drive code of the robot. This function does not return.
// A template type is used in order to facilitate the `virtual_controller` type provided by the recording system.
template<typename T> void drive(T &controller) {

	while (true) {
		int32_t turnPower = controller.get_analog(ANALOG_RIGHT_X);
		int32_t rawForwardPower = controller.get_analog(ANALOG_LEFT_Y);
		// create an exponential easing curve rather than a default linear curve
		// 0.00787401574 is 1/127: we convert it to a domain of [-1, 1] so the exponentiation creates the curve properly
		// sgn call returns the sign of the original -- ensures it is preserved after exponentiation
		int32_t forwardPower = powf(rawForwardPower * 0.00787401574, 2) * 127 * sgn(rawForwardPower);

		// add deadzone to controller analogs
		if (abs(turnPower) < deadzone) turnPower = 0;
		if (abs(forwardPower) < deadzone) forwardPower = 0;
 
		if (turnPower != 0 || forwardPower != 0)
		{
			rightMotors.move((turnPower - forwardPower) * currentSpeed);
			leftMotors.move((turnPower + forwardPower) * currentSpeed);
		}
		else
		{
			rightMotors.brake();
			leftMotors.brake();
		}

		if (controller.get_digital_new_press(DIGITAL_A)) {
			wings.toggle();
		}

		if (controller.get_digital(DIGITAL_R1)) {
			intakeMotor.move(-127);
			lowerRoller.move(127);
			upperRoller.move(-127);
		} else if (controller.get_digital(DIGITAL_R2)) {
			intakeMotor.move(-127);
			lowerRoller.move(127);
			upperRoller.move(127);
		} else if (controller.get_digital(DIGITAL_L2)) {
			intakeMotor.move(127);
			lowerRoller.move(-127);
			upperRoller.move(-127);
		} else if (controller.get_digital(DIGITAL_L1)) {
			intakeMotor.move(-127);
			lowerRoller.move(127);
		} else {
			lowerRoller.brake();
			intakeMotor.brake();
			upperRoller.brake();
			if (controller.get_digital(DIGITAL_UP)) {
				lowerRoller.move(127);
			} else if (controller.get_digital(DIGITAL_DOWN)) {
				lowerRoller.move(-127);
			}

			if (controller.get_digital(DIGITAL_RIGHT)) {
				upperRoller.move(127);
			} else if (controller.get_digital(DIGITAL_LEFT)) {
				upperRoller.move(-127);
			}
		}

		pros::delay(5);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// create a gif of tenna (from deltarune) spinning around
	// reasoning: silly
	init_tenna();

	gif_img = lv_image_create(lv_screen_active());

	lv_obj_center(gif_img);

	lv_timer_create(update_gif, 200, nullptr);

	// set the proper brake mode types for the motors
	leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
	rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	leftMotors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode_all(E_MOTOR_BRAKE_COAST);

	stop_recording();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	drive(*begin_playback("auton_12-12-25_right_modified"));
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	#ifdef RECORDING
		start_recording("auton_12-12-25_right", 20, nullptr);
	#endif
	drive(controllerMaster);
}