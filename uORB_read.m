% uORB_read - Mask initialization for uORB topic reader blocks
%
% This mask class configures the mask initialization function for a Simulink subsystem
% that reads (subscribes to) a single uORB topic and outputs its current value.
%
% Block interface:
%   Parameters:
%     - uorb_topic (string): PX4 topic name (snake_case, auto-populated dropdown)
%     - sample_time (string): Simulink sample time (e.g., '-1' for inherited)
%
%   Output port (Out1):
%     - Type: Bus (named <uorb_topic>_s)
%     - Output of read_<uorb_topic>() C function
%
% Functionality:
%   1. Maps block to read_<uorb_topic>() C function (return-by-value)
%   2. Configures output port data type to match topic structure
%   3. Updates dropdown choices when topic parameter is edited
%
% Generated C code:
%   struct <topic>_s read_<topic>(void)
%     - Returns latest uORB topic value (by value)
%     - Subscribe on first call, check for updates on subsequent calls
%
% Usage:
%   1. Place block from Simulink library
%   2. Select uorb_topic from dropdown
%   3. Block automatically configures C function call
%   4. Connect output to other blocks using same topic type
%   5. Sample time controls how often uORB is checked for new data

classdef uORB_read
    methods(Static)
        % maskInitContext properties available:
        %  - BlockHandle: Handle to the mask (this block)
        %  - MaskObject: Simulink mask object
        %  - MaskWorkspace: Access to mask parameter values
        
        function MaskInitialization(maskInitContext)
            % Primary mask initialization function.
            %
            % Configures the C Caller block to call read_<uorb_topic>() function.
            % Sets the output port data type to the corresponding bus structure.
            % Only applies configuration if model is not locked (allows editing).
            
            apiInstance = px4API(); % Enforce constructor validation and sync checks

            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);

            % Unpack user selection from the active mask parameters table
            uorb_topic = get_param(blockHandle, 'uorb_topic');
            sample_time_val = get_param(blockHandle, 'sample_time');

            % Escape gracefully if block is freshly placed and unconfigured
            if isempty(uorb_topic) || strcmp(uorb_topic, '<empty>') || isempty(strtrim(uorb_topic))
                return;
            end

            % Native PX4 snake_case naming - no conversions needed
            bus_name = [uorb_topic, '_s'];

            c_caller_path = [blockPath '/C_Caller'];
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);

            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % Dynamically map the C Caller to the return-by-value function name.
                    % Simulink handles ports, definitions, and pins automatically.
                    set_param(c_caller_path, 'FunctionName', ['read_' uorb_topic]);

                    % Explicitly specify the output data type as the bus type.
                    % This allows Simulink to properly resolve the structure definition.
                    set_param(outport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);
                catch
                end
            end
        end

        function uorb_topic(callbackContext)
            % Mask parameter callback - updates dropdown choices when user edits topic parameter.
            %
            % Called whenever the 'uorb_topic' parameter value changes.
            % Refreshes the list of available topics from px4API.
            blockHandle = callbackContext.BlockHandle;
            maskObj = Simulink.Mask.get(blockHandle);
            choices = strsplit(px4API.getTopicDropdownString(), ',');
            
            paramObj = maskObj.getParameter('uorb_topic');
            if ~isempty(paramObj)
                paramObj.TypeOptions = choices;
            end
        end
    end
end
