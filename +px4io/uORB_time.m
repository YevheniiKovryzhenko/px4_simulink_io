% /****************************************************************************
%  *
%  *    Copyright (C) 2026  Yevhenii Kovryzhenko. All rights reserved.
%  *
%  *    This program is free software: you can redistribute it and/or modify
%  *    it under the terms of the GNU Affero General Public License as published by
%  *    the Free Software Foundation, either version 3 of the License, or
%  *    (at your option) any later version.
%  *
%  *    This program is distributed in the hope that it will be useful,
%  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
%  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  *    GNU Affero General Public License Version 3 for more details.
%  *
%  *    You should have received a copy of the
%  *    GNU Affero General Public License Version 3
%  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
%  *
%  *    1. Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions, and the following disclaimer.
%  *    2. Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions, and the following disclaimer in
%  *       the documentation and/or other materials provided with the
%  *       distribution.
%  *    3. No ownership or credit shall be claimed by anyone not mentioned in
%  *       the above copyright statement.
%  *    4. Any redistribution or public use of this software, in whole or in part,
%  *       whether standalone or as part of a different project, must remain
%  *       under the terms of the GNU Affero General Public License Version 3,
%  *       and all distributions in binary form must be accompanied by a copy of
%  *       the source code, as stated in the GNU Affero General Public License.
%  *
%  ****************************************************************************/

% uORB_time - Mask initialization for PX4 system time reader block
%
% This mask class configures the mask initialization function for a Simulink subsystem
% that reads the PX4 high-resolution system timer (hrt_absolute_time).
%
% Block interface:
%   Parameters:
%     - sample_time (string): Simulink sample time (e.g., '-1' for inherited)
%
%   Output port (Out1):
%     - Type: uint64
%     - Output of read_px4_system_time() C function (microseconds)
%
% Functionality:
%   1. Maps block to read_px4_system_time() C function (no arguments)
%   2. Configures output port data type to uint64
%   3. Outputs system time in microseconds (hrt_absolute_time)
%
% Generated C code:
%   uint64_t read_px4_system_time(void)
%     - Returns hrt_absolute_time() in microseconds
%     - Useful for elapsed time calculations, synchronization
%
% Usage:
%   1. Place block from Simulink library
%   2. No configuration needed (no topic selection)
%   3. Output is uint64 timestamp
%   4. Connect to Simulink timing/sync blocks as needed
%
% Note: This block has no configurable parameters beyond sample_time.
% The system time function is stateless and always returns current time.

classdef uORB_time
    methods(Static)
        function MaskInitialization(maskInitContext)
            % Primary mask initialization function.
            %
            % Configures the C Caller block to call read_px4_system_time() function.
            % Sets the output port data type to uint64 (high-resolution timer).
            % No configuration parameters needed (no topic selection).
            % Only applies configuration if model is not locked (allows editing).

            px4io.px4API(); % Enforce constructor validation and sync checks

            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);

            sample_time_val = get_param(blockHandle, 'sample_time');

            c_caller_path = [blockPath '/C_Caller'];
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);

            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                % Configure the C Caller to call high-resolution timer function.
                set_param(c_caller_path, 'FunctionName', 'read_px4_system_time');
                set_param(outport_path, 'OutDataTypeStr', 'uint64');
            end
        end
    end
end