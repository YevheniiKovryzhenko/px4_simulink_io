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

% param_read - Mask initialization for PX4 parameter reader blocks
%
% This mask class configures the mask initialization function for a Simulink subsystem
% that reads a PX4 parameter value by name (string).
%
% Block interface:
%   Parameters:
%     - param_name (string): PX4 parameter name (e.g., 'SYS_AUTOSTART')
%     - sample_time (string): Simulink sample time (e.g., '-1' for inherited)
%     - param_type (string): Parameter data type ('int32', 'float', 'single')
%
%   Inputs:
%     - Parameter name (String Constant or From Workspace block)
%       Connect a string input that provides the parameter name at runtime
%
%   Output port (Out1):
%     - Type: int32 or single (based on param_type)
%     - Output of read_px4_param_int32() or read_px4_param_float() C function
%
% Functionality:
%   1. Selects appropriate read function based on param_type
%   2. Configures output port data type (int32 or single)
%   3. Parameter name is passed from connected input block
%   4. Performs runtime type checking and validation
%
% Generated C code:
%   int32_t read_px4_param_int32(const char* param_name)
%   float read_px4_param_float(const char* param_name)
%     - Looks up parameter by name (param_find)
%     - Validates parameter type matches
%     - Returns value or 0/NaN on error
%
% Usage:
%   1. Place block from Simulink library
%   2. Select parameter type (int32 or float)
%   3. Connect String Constant block to parameter name input
%   4. Output is the parameter value
%   5. Use in control loops, tuning, initialization
%
% Note: Parameter name must be a valid PX4 parameter.
% Returns 0 (int32) or NaN (float) if parameter not found or wrong type.

classdef param_read
    methods(Static)
        % maskInitContext properties available:
        %  - BlockHandle: Handle to the mask (this block)
        %  - MaskObject: Simulink mask object
        %  - MaskWorkspace: Access to mask parameter values
        
        function MaskInitialization(maskInitContext)
            % Primary mask initialization function.
            %
            % Configures C Caller block to use the appropriate parameter read function
            % (read_px4_param_int32 or read_px4_param_float) based on param_type.
            % Sets output port data type to match parameter type.
            
            px4io.px4API();

            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);

            param_name = get_param(blockHandle, 'param_name');
            sample_time_val = get_param(blockHandle, 'sample_time');
            param_type = px4io.param_read.getParamDatatype(blockHandle);
            param_type = px4io.param_read.normalizeParamDatatype(param_type);

            if isempty(param_name) || strcmp(param_name, '<empty>') || isempty(strtrim(param_name))
                return;
            end

            if strcmp(param_type, 'int32')
                function_name = 'read_px4_param_int32';
                out_data_type = 'int32';
            else
                function_name = 'read_px4_param_float';
                out_data_type = 'single';
            end

            c_caller_path = [blockPath '/C_Caller'];
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);

            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    set_param(c_caller_path, 'FunctionName', function_name);

                    set_param(outport_path, 'OutDataTypeStr', out_data_type);
                catch
                end
            end
        end

        function paramType = getParamDatatype(blockHandle)
            % Retrieve parameter data type from mask parameter (with fallback).
            %
            % Attempts to read 'param_type' parameter, falling back to 'datatype'.
            % Returns default 'int32' if neither is found.
            %
            % Input:
            %   blockHandle - Simulink block handle
            %
            % Output:
            %   paramType - Data type string ('int32', 'float32', 'single', etc.)
            
            paramType = 'int32';
            try
                paramType = get_param(blockHandle, 'param_type');
            catch
                try
                    paramType = get_param(blockHandle, 'datatype');
                catch
                end
            end
        end

        function paramType = normalizeParamDatatype(paramType)
            % Normalize parameter data type to canonical form.
            %
            % Converts various data type naming conventions to standard forms:
            %   - 'float32', 'single' -> 'float' (uses read_px4_param_float)
            %   - All others -> 'int32' (uses read_px4_param_int32)
            %
            % Input:
            %   paramType - Raw parameter type string
            %
            % Output:
            %   paramType - Normalized type ('float' or 'int32')
            
            paramType = lower(strtrim(string(paramType)));
            if paramType == "single" || paramType == "float32"
                paramType = 'float';
            else
                paramType = 'int32';
            end
        end
    end
end
