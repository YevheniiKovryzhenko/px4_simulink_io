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

#pragma once

#include <px4_platform_common/module.h>

// Include the auto-generated model-agnostic wrapper (works with any model name)
#include "simulink_model_wrapper.h"

extern "C" __EXPORT int simulink_io_main(int argc, char *argv[]);

class SimulinkIO : public ModuleBase<SimulinkIO>
{
public:
    SimulinkIO() = default;
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
    // Instantiate your generated Simulink model using the model-agnostic wrapper
    SimulinkWrapper::SimulinkModel _simulink_model;
};
