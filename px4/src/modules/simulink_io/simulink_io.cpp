#include "simulink_io.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

SimulinkIO::SimulinkIO() :
    ModuleParams(nullptr)
{
    // Run any internal native module setups here
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
    uint32_t iteration_counter = 0;

    // Check parameters on boot
    parameters_update(true);

    while (!should_exit()) {
        // Linearly increment tick baseline target
        loop_time_reference += interval_us;

        // Execute the generated algorithm code
        _simulink_model.step();

        // Increment loop tick counter
        iteration_counter++;

        // PROTECTED: This block now runs exactly every 2 seconds (400 cycles @ 200Hz)
        if (iteration_counter >= 400) {
            PX4_INFO("[Simulink Test] Class is ticking healthy! Running background steps...");

            // Query fields out of the generated model's global Output variable structure (Test_Y).
            // Extract a read-only handle reference to the private data structure
            const Test::ExtY_Test_T &outputs = _simulink_model.getExternalOutputs();

            PX4_INFO("[Simulink Test] Local Position: %f, %f, %f",
                     (double)outputs.Out1.x,
                     (double)outputs.Out1.y,
                     (double)outputs.Out1.z);

            PX4_INFO("[Simulink Test] Local Position Setpoint: %f, %f, %f",
                     (double)outputs.Out2.x,
                     (double)outputs.Out2.y,
                     (double)outputs.Out2.z);

            PX4_INFO("[Simulink Test] Parameter Read: %f",
                     (double)outputs.Out3);

            iteration_counter = 0; // Reset counter
        }

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
