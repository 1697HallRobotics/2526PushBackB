#ifndef _RECORDING_H_
#define _RECORDING_H_

#include <fstream>
#include <string>
#include <vector>
#include <deque>

#include "api.h"

using namespace std;
using namespace chrono;
using namespace pros;
using namespace pros::c;

#define DEBUG 0

typedef struct //Controller data
{
    signed char axis[4];

    signed char digital[12];
} ControllerData;

typedef struct
{
    double positionX;
    double positionY;
    double heading;
} PositionData;

class virtual_controller_axis
{
public:
    signed char position_value = 0;
    signed int position();
};

class virtual_controller_digital
{
public:
    signed char pressing_value = 0;
    bool pressing();
};

/**
 * @brief A class that simulates the controller. Provides facades for both VEXCode and PROS bindings, so they can be easily swapped out 
 */
class virtual_controller
{
public:
    // this provides support for both vexcode and pros implementations
    virtual_controller_digital PrevButtonA, PrevButtonB, PrevButtonX, PrevButtonY, PrevButtonUp, PrevButtonDown, PrevButtonLeft, PrevButtonRight, PrevButtonL1, PrevButtonL2, PrevButtonR1, PrevButtonR2;

    virtual_controller_axis Axis1, Axis2, Axis3, Axis4;
    virtual_controller_digital ButtonA, ButtonB, ButtonX, ButtonY, ButtonUp, ButtonDown, ButtonLeft, ButtonRight, ButtonL1, ButtonL2, ButtonR1, ButtonR2;

    void copy_old();
    int32_t get_analog(controller_analog_e_t channel);
    int32_t get_digital(controller_digital_e_t button);
    int32_t get_digital_new_press(controller_digital_e_t button);
};

// the current recording stream
static ofstream recording_output_stream;
// the currently saved captures of the controller that have not been flushed to the disk
static vector<ControllerData> recording_buffer;
// the recording time of the current recording, in seconds
static uint32_t max_recording_time;

// the virtual controller currently held by the playback
static virtual_controller* playback_controller;
// the currently unplayed data in the file
static deque<ControllerData> playback_buffer;

// Stop the recording or playback at the next process frame.
static bool stop_system = false;

/**
 * @brief Start the recording.
 * @param filename The name of the file. Automatically appends a .vrf extension to the file.
 * @param length The length of the recording, in seconds.
 * @param gps An options GPS to pass in, in order to record the positional data to be retrieved.
 */
void start_recording(const string filename, uint8_t length, Gps* gps);
/**
 * @brief The recording thread of the recording system. Typically used by the `start_recording` method.
 * @param param An argument required by the PROS RTOS task. Unused, just pass `nullptr` into it.
 * @warning Only call if you know what you are doing, this is a blocking call!
 */
void recording_thread(void* param);
/**
 * @brief Immediately end the recording and clean it up.
 */
void stop_recording();

/**
 * @brief Get the position data of a recording system.
 * @return A struct containing the 
 */
PositionData get_position(string filename);

/**
 * @brief Begin the playback.
 * @return An pointer to the virtual controller, being updated in real time in accordance to the playback timing.
 */
virtual_controller* begin_playback(string filename);
/**
 * @brief The playback thread of the recording system. Typically used by the `begin_playback` method.
 * @param param An argument required by the PROS RTOS task. Unused, just pass `nullptr` into it.
 */
void playback_thread(void* param);
/**
 * @brief Immediately end the playback and clean it up.
 */
void stop_playback();

#endif