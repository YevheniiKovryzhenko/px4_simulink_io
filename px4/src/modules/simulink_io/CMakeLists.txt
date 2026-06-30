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

file(GLOB SIMULINK_GENERATED_SRCS "generated_code/*.cpp" "generated_code/*.c")


px4_add_module(
    MODULE modules__simulink_io
    MAIN simulink_io
    STACK_MAIN 2500
    SRCS
        simulink_io.cpp
        ${SIMULINK_GENERATED_SRCS}
    INCLUDES
        generated_code

    MODULE_CONFIG
		module.yaml
        module_stock_missing.yaml
)


set_source_files_properties(${SIMULINK_GENERATED_SRCS}
    PROPERTIES COMPILE_FLAGS "-Wno-float-equal -Wno-logical-op -Wno-unused-variable -Wno-unused-but-set-variable -Wno-parentheses"
)
