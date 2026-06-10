% param_write - Mask initialization for PX4 parameter writer blocks
%
% This mask class configures the mask initialization function for a Simulink subsystem
% that writes a PX4 parameter value by name (string).
%
% Block interface:
%   Parameters:
%     - param_name (string): PX4 parameter name (e.g., 'SYS_AUTOSTART')
%     - sample_time (string): Simulink sample time (e.g., '-1' for inherited)
%     - param_type (string): Parameter data type ('int32', 'float', 'single')
%
%   Inputs:
%     - In1: Parameter value (int32 or single)
%     - Parameter name (String Constant or From Workspace block)
%       Connect a string input that provides the parameter name at runtime
%
% Functionality:
%   1. Selects appropriate write function based on param_type
%   2. Configures input port data type (int32 or single)
%   3. Parameter name is passed from connected input block
%   4. Only updates if value has actually changed (optimized)
%
% Generated C code:
%   void write_px4_param_int32(const char* param_name, int32_t value)
%   void write_px4_param_float(const char* param_name, float value)
%     - Looks up parameter by name (param_find)
%     - Validates parameter type matches
%     - Reads current value, only calls param_set if changed
%     - Uses 1e-6f epsilon for float comparisons
%
% Optimization:
%   - Block can be called at fixed rate (e.g., 50 Hz) safely
%   - Only actually updates PX4 parameter when value changes
%   - Avoids unnecessary system notifications and syncs
%   - Reduces computational overhead on PX4
%
% Usage:
%   1. Place block from Simulink library
%   2. Select parameter type (int32 or float)
%   3. Connect value input (e.g., slider, constant, calculation)
%   4. Connect parameter name input (e.g., String Constant)
%   5. Sample time controls update frequency
%   6. Use for tuning, parameter sweeps, adaptive control
%
% Note: Parameter name must be a valid PX4 parameter.
% Write fails silently if parameter not found or wrong type.

classdef param_write
    methods(Static)
        % maskInitContext properties available:
        %  - BlockHandle: Handle to the mask (this block)
        %  - MaskObject: Simulink mask object
        %  - MaskWorkspace: Access to mask parameter values
        
        function MaskInitialization(maskInitContext)
            % Primary mask initialization function.
            %
            % Configures C Caller block to use the appropriate parameter write function
            % (write_px4_param_int32 or write_px4_param_float) based on param_type.
            % Sets input port data type to match parameter type.
            
            px4io.px4API();

            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);

            param_name = get_param(blockHandle, 'param_name');
            sample_time_val = get_param(blockHandle, 'sample_time');
            param_type = px4io.param_write.getParamDatatype(blockHandle);
            param_type = px4io.param_write.normalizeParamDatatype(param_type);

            if isempty(param_name) || strcmp(param_name, '<empty>') || isempty(strtrim(param_name))
                return;
            end

            if strcmp(param_type, 'int32')
                function_name = 'write_px4_param_int32';
                in_data_type = 'int32';
            else
                function_name = 'write_px4_param_float';
                in_data_type = 'single';
            end

            c_caller_path = [blockPath '/C_Caller'];
            inport_path = [blockPath '/In1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(inport_path, 'SampleTime', sample_time_val);

            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    set_param(c_caller_path, 'FunctionName', function_name);

                    set_param(inport_path, 'OutDataTypeStr', in_data_type);
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
            %   - 'float32', 'single' -> 'float' (uses write_px4_param_float)
            %   - All others -> 'int32' (uses write_px4_param_int32)
            %
            % Important: Value change detection uses appropriate comparison:
            %   - int32: Direct equality check
            %   - float: Epsilon-based check (1e-6f) to avoid floating-point artifacts
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
