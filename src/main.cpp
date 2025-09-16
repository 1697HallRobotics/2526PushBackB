#include "main.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_timer.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "pros/misc.h"
#include "tenna_gif.h"

static lv_obj_t* gif_img;
int gifTimer = 0;

void update_gif(lv_timer_t* timer) {
	lv_image_set_src(gif_img, &(tenna_gif[(gifTimer) % 7]));
	gifTimer++;
	lv_obj_invalidate(gif_img);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	init_tenna();

	gif_img = lv_image_create(lv_screen_active());

	lv_obj_center(gif_img);

	lv_timer_create(update_gif, 200, nullptr);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous() {}

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
	while (true) {
		int32_t turnPower = controller.get_analog(ANALOG_RIGHT_X);
		int32_t rawForwardPower = controller.get_analog(ANALOG_LEFT_Y);
		int32_t forwardPower = powf(rawForwardPower * 0.00787401574, 2) * 127;

		if (abs(turnPower) < deadzone) turnPower = 0;
		if (abs(forwardPower) < deadzone) forwardPower = 0;

		if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			currentSpeed = speed * 2;
		}
		else 
		{
			currentSpeed = speed;
		}

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
		
		pros::delay(5);
	}
}