#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

// Include the auto-generated model-agnostic wrapper (works with any model name)
#include "simulink_model_wrapper.h"

// CRUCIAL FIX: Bring the time literal operator conversions into scope
using namespace time_literals;

extern "C" __EXPORT int simulink_io_main(int argc, char *argv[]);

class SimulinkIO : public ModuleBase<SimulinkIO>, public ModuleParams
{
public:
    SimulinkIO();
    virtual ~SimulinkIO() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static SimulinkIO *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:
    void parameters_update(bool force = false);

    // Instantiate your generated Simulink model using the model-agnostic wrapper
    SimulinkWrapper::SimulinkModel _simulink_model;

    // Standard parameter framework handle tracking framework overrides
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
