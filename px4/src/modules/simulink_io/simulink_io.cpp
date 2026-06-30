# /****************************************************************************
#  *
#  *    Copyright (C) 2025  Yevhenii Kovryzhenko. All rights reserved.
#  *
#  *    This program is free software: you can redistribute it and/or modify
#  *    it under the terms of the GNU Affero General Public License as published by
#  *    the Free Software Foundation, either version 3 of the License, or
#  *    (at your option) any later version.
#  *
#  *    This program is distributed in the hope that it will be useful,
#  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
#  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  *    GNU Affero General Public License Version 3 for more details.
#  *
#  *    You should have received a copy of the
#  *    GNU Affero General Public License Version 3
#  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#  *
#  *    1. Redistributions of source code must retain the above copyright
#  *       notice, this list of conditions, and the following disclaimer.
#  *    2. Redistributions in binary form must reproduce the above copyright
#  *       notice, this list of conditions, and the following disclaimer in
#  *       the documentation and/or other materials provided with the
#  *       distribution.
#  *    3. No ownership or credit shall be claimed by anyone not mentioned in
#  *       the above copyright statement.
#  *    4. Any redistribution or public use of this software, in whole or in part,
#  *       whether standalone or as part of a different project, must remain
#  *       under the terms of the GNU Affero General Public License Version 3,
#  *       and all distributions in binary form must be accompanied by a copy of
#  *       the source code, as stated in the GNU Affero General Public License.
#  *
#  ****************************************************************************/

#include "simulink_io.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

SimulinkIO::SimulinkIO() :
    ModuleParams(nullptr)
{
    // Run any internal native module setups here
}

#include <cstdio>
#include <cmath>

// Reuse or keep the print_row_vector function from the previous step
void print_row_vector(const char* label, const float* data, int size, int max_line_len = 75) {
    char hdr_buf[128];
    char val_buf[128];
    int hdr_pos = snprintf(hdr_buf, sizeof(hdr_buf), "%-9s", label);
    int val_pos = snprintf(val_buf, sizeof(val_buf), "%-9s", "Vals: ");
    const int COL_WIDTH = 9;

    for (int i = 0; i < size; i++) {
        if (hdr_pos + COL_WIDTH >= max_line_len) {
            PX4_INFO("%s", hdr_buf);
            PX4_INFO("%s", val_buf);
            hdr_pos = snprintf(hdr_buf, sizeof(hdr_buf), "%-9s", label);
            val_pos = snprintf(val_buf, sizeof(val_buf), "%-9s", "Vals: ");
        }
        char item_hdr[16];
        if (size <= 3) {
            snprintf(item_hdr, sizeof(item_hdr), "%s.%c", label, (i == 0) ? 'x' : (i == 1) ? 'y' : 'z');
        } else {
            snprintf(item_hdr, sizeof(item_hdr), "%s[%d]", label, i);
        }
        int h_written = snprintf(hdr_buf + hdr_pos, sizeof(hdr_buf) - hdr_pos, "%-9s", item_hdr);
        int v_written = snprintf(val_buf + val_pos, sizeof(val_buf) - val_pos, "%-9.3f", static_cast<double>(data[i]));
        if (h_written > 0 && v_written > 0) {
            hdr_pos += h_written;
            val_pos += v_written;
        }
    }
    if (hdr_pos > 9) {
        PX4_INFO("%s", hdr_buf);
        PX4_INFO("%s", val_buf);
    }
}

void print_quaternion(const QuaternionAttitude& q) {
    PX4_INFO("--- Quaternion Attitude ---");
    char hdr[64] = "Quat:    ";
    char val[64] = "Vals:    ";

    snprintf(hdr + 9, sizeof(hdr) - 9, "%-8s%-8s%-8s%-8s", "w", "x", "y", "z");
    snprintf(val + 9, sizeof(val) - 9, "%-8.3f%-8.3f%-8.3f%-8.3f", static_cast<double>(q.w), static_cast<double>(q.x), static_cast<double>(q.y), static_cast<double>(q.z));

    PX4_INFO("%s", hdr);
    PX4_INFO("%s", val);
}

// Helper to print a 3x3 Rotation Tensor cleanly
void print_rotation_matrix(const char* name, const float matrix[9]) {
    PX4_INFO("--- %s Matrix ---", name);
    for (int row = 0; row < 3; row++) {
        char row_buf[64];
        snprintf(row_buf, sizeof(row_buf), "[ %6.3f  %6.3f  %6.3f ]",
                 static_cast<double>(matrix[row * 3 + 0]),
                 static_cast<double>(matrix[row * 3 + 1]),
                 static_cast<double>(matrix[row * 3 + 2]));
        PX4_INFO("%s", row_buf);
    }
}

void print_common_states(const char* prefix, const CommonStates& cs) {
    char sub_label[32];
    snprintf(sub_label, sizeof(sub_label), "%s.Pos", prefix);  print_row_vector(sub_label, cs.Position, 3);
    snprintf(sub_label, sizeof(sub_label), "%s.Vel", prefix);  print_row_vector(sub_label, cs.Velocity, 3);
    snprintf(sub_label, sizeof(sub_label), "%s.Acc", prefix);  print_row_vector(sub_label, cs.Acceleration, 3);
    snprintf(sub_label, sizeof(sub_label), "%s.wVel", prefix); print_row_vector(sub_label, cs.AngularVelocity, 3);
    snprintf(sub_label, sizeof(sub_label), "%s.wAcc", prefix); print_row_vector(sub_label, cs.AngularAcceleration, 3);
}

void print_euler(const EulerAttitude& euler) {
    char hdr[64];
    char val[64];
    snprintf(hdr, sizeof(hdr), "Euler:   %-9s%-9s%-9s", "Roll", "Pitch", "Yaw");
    snprintf(val, sizeof(val), "Vals:    %-9.3f%-9.3f%-9.3f", static_cast<double>(euler.Roll), static_cast<double>(euler.Pitch), static_cast<double>(euler.Yaw));
    PX4_INFO("%s", hdr);
    PX4_INFO("%s", val);
}

void print_global_position(const GlobalPositionPoint& gp) {
    // Print LLA coordinates
    char lla_hdr[64];
    char lla_val[64];
    snprintf(lla_hdr, sizeof(lla_hdr), "LLA:     %-12s%-12s%-12s", "Lat", "Lon", "Alt");
    snprintf(lla_val, sizeof(lla_val), "Vals:    %-12.6f%-12.6f%-12.3f", static_cast<double>(gp.LLA.Latitude), static_cast<double>(gp.LLA.Longitude), static_cast<double>(gp.LLA.Altitude));
    PX4_INFO("%s", lla_hdr);
    PX4_INFO("%s", lla_val);

    // Print derived velocities/states
    print_row_vector("GP.Vel", gp.Velocity, 3);

    char extra_hdr[64];
    char extra_val[64];
    snprintf(extra_hdr, sizeof(extra_hdr), "GP.Misc: %-9s%-9s%-9s%-9s", "GndSpd", "Course", "TerrAlt", "Contact");
    snprintf(extra_val, sizeof(extra_val), "Vals:    %-9.3f%-9.3f%-9.3f%-9s",
             static_cast<double>(gp.GroundSpeed), static_cast<double>(gp.Course), static_cast<double>(gp.TerrainAltitude), gp.GroundContact ? "true" : "false");
    PX4_INFO("%s", extra_hdr);
    PX4_INFO("%s", extra_val);
}

void print_air_data(const AirData& air) {
    char p1_hdr[64]; char p1_val[64];
    snprintf(p1_hdr, sizeof(p1_hdr), "Aero:    %-9s%-9s%-9s%-9s", "AoA", "Beta", "Airspeed", "Temp");
    snprintf(p1_val, sizeof(p1_val), "Vals:    %-9.3f%-9.3f%-9.3f%-9.3f", static_cast<double>(air.AoA), static_cast<double>(air.Beta), static_cast<double>(air.Airspeed), static_cast<double>(air.Temperature));
    PX4_INFO("%s", p1_hdr);
    PX4_INFO("%s", p1_val);

    char p2_hdr[64]; char p2_val[64];
    snprintf(p2_hdr, sizeof(p2_hdr), "Press:   %-9s%-9s", "Static", "Dynamic");
    snprintf(p2_val, sizeof(p2_val), "Vals:    %-9.1f%-9.1f", static_cast<double>(air.StaticPressure), static_cast<double>(air.DynamicPressure));
    PX4_INFO("%s", p2_hdr);
    PX4_INFO("%s", p2_val);

    print_row_vector("Air.RelV", air.VelocityRelativeBody, 3);
}

void print_states(const States& state) {
    PX4_INFO("=================================================");
    PX4_INFO("                VEHICLE STATES                   ");
    PX4_INFO("=================================================");
    PX4_INFO("Timestamp (double): %.4f", static_cast<double>(state.time));

    PX4_INFO("--- Inertial States ---");
    print_common_states("Inert", state.Inertial);

    PX4_INFO("--- Body-Fixed States ---");
    print_common_states("Body", state.Body);

    PX4_INFO("--- Orientation States ---");
    // Reuses the print_quaternion helper function defined in the previous step
    print_quaternion(state.Quaternion);
    print_euler(state.Euler);

    print_rotation_matrix("Inertial to Body", state.Rotation.Inertial2Body);
    print_rotation_matrix("Body to Inertial", state.Rotation.Body2Inertial);

    PX4_INFO("--- Global Position ---");
    print_global_position(state.GlobalPosition);

    PX4_INFO("--- Air Data / Aerodynamics ---");
    print_air_data(state.Air);
}

void print_trajectory(const TrajectoryPoint& traj) {
    PX4_INFO("--- Trajectory ---");
    print_row_vector("Pos", traj.Position, 3);
    print_row_vector("Vel", traj.Velocity, 3);
    print_row_vector("Acc", traj.Acceleration, 3);
    print_row_vector("Jrk", traj.Jerk, 3);
    print_row_vector("Snp", traj.Snap, 3);

    // Package Yaw terms into a single row to fit perfectly
    char yaw_hdr[128] = "Yaw Terms:";
    char yaw_val[128] = "Vals:     ";
    int h_pos = 10, v_pos = 10;

    const char* y_names[] = { "Yaw", "Rate", "Acc", "Jrk", "Snp", "Done" };
    char y_vals[6][10];
    snprintf(y_vals[0], sizeof(y_vals[0]), "%-9.3f", static_cast<double>(traj.Yaw));
    snprintf(y_vals[1], sizeof(y_vals[1]), "%-9.3f", static_cast<double>(traj.YawRate));
    snprintf(y_vals[2], sizeof(y_vals[2]), "%-9.3f", static_cast<double>(traj.YawAcceleration));
    snprintf(y_vals[3], sizeof(y_vals[3]), "%-9.3f", static_cast<double>(traj.YawJerk));
    snprintf(y_vals[4], sizeof(y_vals[4]), "%-9.3f", static_cast<double>(traj.YawSnap));
    snprintf(y_vals[5], sizeof(y_vals[5]), "%-9s", traj.Completed ? "true" : "false");

    for(int i = 0; i < 6; i++) {
        h_pos += snprintf(yaw_hdr + h_pos, sizeof(yaw_hdr) - h_pos, "%-9s", y_names[i]);
        v_pos += snprintf(yaw_val + v_pos, sizeof(yaw_val) - v_pos, "%-9s", y_vals[i]);
    }
    PX4_INFO("%s", yaw_hdr);
    PX4_INFO("%s", yaw_val);
}

void print_control_6dof(const Control6DoF& ctrl) {
    PX4_INFO("--- Control 6DoF Output ---");
    print_row_vector("Force", ctrl.Force, 3);
    print_row_vector("Moment", ctrl.Moment, 3);
}

void print_control_references(const ControlReferences& refs) {
    PX4_INFO("=================================================");
    PX4_INFO("               CONTROL REFERENCES                ");
    PX4_INFO("=================================================");

    print_trajectory(refs.Trajectory);
    print_quaternion(refs.Quaternion);

    PX4_INFO("--- Angular States ---");
    print_row_vector("AngVel", refs.AngularVelocity, 3);
    print_row_vector("AngAcc", refs.AngularAcceleration, 3);

    print_control_6dof(refs.ControlOutput);
}


void print_pilot_input(Sticks PilotInput)
{
    PX4_INFO("=================================================");
    PX4_INFO("                 PILOT INPUT                     ");
    PX4_INFO("=================================================");

    const char* names[] = { "Roll", "Pitch", "Yaw", "Throt", "Mode", "Armed", "Aux1", "Aux2", "Aux3", "Aux4", "Aux5", "Aux6" };
    char val_str[12][10];

    // Pre-format values to standard 9-character width strings
    snprintf(val_str[0], sizeof(val_str[0]), "%-9.3f", static_cast<double>(PilotInput.Roll));
    snprintf(val_str[1], sizeof(val_str[1]), "%-9.3f", static_cast<double>(PilotInput.Pitch));
    snprintf(val_str[2], sizeof(val_str[2]), "%-9.3f", static_cast<double>(PilotInput.Yaw));
    snprintf(val_str[3], sizeof(val_str[3]), "%-9.3f", static_cast<double>(PilotInput.Throttle));
    snprintf(val_str[4], sizeof(val_str[4]), "%-9d",   (int)PilotInput.FlightMode);
    snprintf(val_str[5], sizeof(val_str[5]), "%-9s",   PilotInput.Armed ? "true" : "false");
    snprintf(val_str[6], sizeof(val_str[6]), "%-9.3f", static_cast<double>(PilotInput.Auxiliary_1));
    snprintf(val_str[7], sizeof(val_str[7]), "%-9.3f", static_cast<double>(PilotInput.Auxiliary_2));
    snprintf(val_str[8], sizeof(val_str[8]), "%-9.3f", static_cast<double>(PilotInput.Auxiliary_3));
    snprintf(val_str[9], sizeof(val_str[9]), "%-9.3f", static_cast<double>(PilotInput.Auxiliary_4));
    snprintf(val_str[10], sizeof(val_str[10]), "%-9.3f", static_cast<double>(PilotInput.Auxiliary_5));
    snprintf(val_str[11], sizeof(val_str[11]), "%-9.3f", static_cast<double>(PilotInput.Auxiliary_6));

    char hdr_buf[128] = "Pilot: ";
    char val_buf[128] = "Vals:  ";
    int hdr_pos = 7;
    int val_pos = 7;

    const int MAX_LINE_LEN = 75; // Enforce safe line wrap limit
    const int COL_WIDTH = 9;

    for (int i = 0; i < 12; i++) {
        // If the next column causes us to exceed the maximum line limit, flush the current line
        if (hdr_pos + COL_WIDTH >= MAX_LINE_LEN) {
            PX4_INFO("%s", hdr_buf);
            PX4_INFO("%s", val_buf);

            // Reset blocks for next wrapped row
            strcpy(hdr_buf, "Pilot: ");
            strcpy(val_buf, "Vals:  ");
            hdr_pos = 7;
            val_pos = 7;
        }

        int h_written = snprintf(hdr_buf + hdr_pos, sizeof(hdr_buf) - hdr_pos, "%-9s", names[i]);
        int v_written = snprintf(val_buf + val_pos, sizeof(val_buf) - val_pos, "%-9s", val_str[i]);

        if (h_written > 0 && v_written > 0) {
            hdr_pos += h_written;
            val_pos += v_written;
        }
    }

    // Print final remaining columns
    if (hdr_pos > 7) {
        PX4_INFO("%s", hdr_buf);
        PX4_INFO("%s", val_buf);
    }
}

void print_actuator_commands(const float ActuatorCommands[16])
{
    char idx_buf[128] = "Mtr idx: ";
    char val_buf[128] = "Values:  ";
    int idx_pos = 9; // Length of "Mtr idx: "
    int val_pos = 9; // Length of "Values:  "

    const int MAX_LINE_LEN = 75; // Enforce safe line wrap limit
    const int COL_WIDTH = 7;

    for (int i = 0; i < 16; i++) {
        float cmd = ActuatorCommands[i];
        if (!std::isnan(cmd)) {
            // Check line constraint before writing characters
            if (idx_pos + COL_WIDTH >= MAX_LINE_LEN) {
                PX4_INFO("%s", idx_buf);
                PX4_INFO("%s", val_buf);

                // Reset blocks for wrapped channels
                strcpy(idx_buf, "Mtr idx: ");
                strcpy(val_buf, "Values:  ");
                idx_pos = 9;
                val_pos = 9;
            }

            int idx_written = snprintf(idx_buf + idx_pos, sizeof(idx_buf) - idx_pos, "%-7d", i);
            int val_written = snprintf(val_buf + val_pos, sizeof(val_buf) - val_pos, "%-7.3f", static_cast<double>(cmd));

            if (idx_written > 0 && val_written > 0) {
                idx_pos += idx_written;
                val_pos += val_written;
            }
        }
    }

    // Print final remaining columns
    if (idx_pos > 9) {
        PX4_INFO("%s", idx_buf);
        PX4_INFO("%s", val_buf);
    }
}


void SimulinkIO::run()
{
    PX4_INFO("Initializing Simulink generated object class...");
    _simulink_model.initialize();

    PX4_INFO("Module initialized successfully. Starting loop at 200Hz...");

    // Setup cyclic tick reference tracking
    hrt_abstime loop_time_reference = hrt_absolute_time();
    const hrt_abstime interval_us = 5000; // 5000 microseconds = 5ms (200Hz)

    // Setup an internal slow iteration counter for our print test
    // uint32_t iteration_counter = 0;

    // Check parameters on boot
    parameters_update(true);

    while (!should_exit()) {
        // Linearly increment tick baseline target
        loop_time_reference += interval_us;

        // Execute the generated algorithm code
        _simulink_model.step();

        // Increment loop tick counter
        // iteration_counter++;

        // This block runs exactly every 1 seconds (200 cycles @ 200Hz)
        // if (iteration_counter >= 200) {
        //     PX4_INFO("[Simulink Test] Class is ticking healthy! Running background steps...");

        //     // Query fields out of the generated model's global Output variable structure (Test_Y).
        //     // Extract a read-only handle reference to the private data structure
        //     const HardwareModel::ExtY_HardwareModel_T &outputs = _simulink_model.getExternalOutputs();
        //     PX4_INFO("\n");
        //     print_pilot_input(outputs.PilotInput);
        //     print_states(outputs.States_c);
        //     print_control_references(outputs.ControlOutputs);
        //     print_actuator_commands(outputs.ActuatorCommands);
        //     PX4_INFO("\n");

        //     iteration_counter = 0; // Reset counter
        // }

        // Check for runtime system parameters updates
        parameters_update();

        // High-precision block sleep calculating the true drift remaining
        hrt_abstime current_time = hrt_absolute_time();
        if (loop_time_reference > current_time) {
            px4_usleep(loop_time_reference - current_time);
        } else {
            // Loop overrun safe fallback: Reset reference timing baseline to catch up
            loop_time_reference = current_time;
        }
    }
}



void SimulinkIO::parameters_update(bool force)
{
    if (_parameter_update_sub.updated() || force) {
        parameter_update_s update;
        _parameter_update_sub.copy(&update);
        updateParams();
    }
}

int SimulinkIO::print_status()
{
    PX4_INFO("Status: ACTIVE");
    PX4_INFO("Execution Rate: 200 Hz");
    return 0;
}

int SimulinkIO::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("simulink_io",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2500,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

SimulinkIO *SimulinkIO::instantiate(int argc, char *argv[])
{
    return new SimulinkIO();
}

int SimulinkIO::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int SimulinkIO::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Custom out-of-tree background module running an auto-generated Simulink C++ model.
Executes deterministically at 200Hz independent of outdated MATLAB toolboxes.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("simulink_io", "control");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int simulink_io_main(int argc, char *argv[])
{
    return SimulinkIO::main(argc, argv);
}
