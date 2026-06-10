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
                try
                    % Configure the C Caller to call high-resolution timer function.
                    set_param(c_caller_path, 'FunctionName', 'read_px4_system_time');
                    set_param(outport_path, 'OutDataTypeStr', 'uint64');
                catch
                end
            end
        end
    end
end
