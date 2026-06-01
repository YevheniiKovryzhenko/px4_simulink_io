% uORB_msg - Mask initialization for uORB message initialization blocks
%
% This mask class configures the mask initialization function for a Simulink subsystem
% that initializes uORB topic structures with zeros (or NaN for float fields).
%
% Block interface:
%   Parameters:
%     - uorb_topic (string): PX4 topic name (snake_case, auto-populated dropdown)
%     - sample_time (string): Simulink sample time (e.g., '-1' for inherited)
%     - init_value (string): Initialization strategy ('Zero' or 'NaN')
%
%   Output port (Out1):
%     - Type: Bus (named <uorb_topic>_s)
%     - Output of init_<uorb_topic>() C function
%
% Functionality:
%   1. Maps block to init_<uorb_topic>() C function
%   2. Sets initialize_to_nan parameter based on init_value selection
%   3. Configures output port data type to match topic structure
%   4. Updates dropdown choices when topic parameter is edited
%
% Generated C code:
%   struct <topic>_s init_<topic>(bool initialize_to_nan)
%     - Returns zero-initialized struct
%     - If initialize_to_nan=true, float fields set to NaN
%
% Usage:
%   1. Place block from Simulink library
%   2. Select uorb_topic from dropdown
%   3. Block automatically configures C function call
%   4. Connect output to other blocks using same topic type

classdef uORB_msg
    methods(Static)
        % maskInitContext properties available:
        %  - BlockHandle: Handle to the mask (this block)
        %  - MaskObject: Simulink mask object
        %  - MaskWorkspace: Access to mask parameter values
        
        function MaskInitialization(maskInitContext)
            apiInstance = px4API(); % Enforce constructor validation and sync checks

            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);

            % Unpack user selections from the active mask parameters table
            uorb_topic = get_param(blockHandle, 'uorb_topic');
            sample_time_val = get_param(blockHandle, 'sample_time'); 
            init_value = get_param(blockHandle, 'init_value'); 

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
                    % 1. Dynamically map the C Caller to your return-by-value initialization function name
                    set_param(c_caller_path, 'FunctionName', ['init_' uorb_topic]);

                    % 2. CANONICAL R2025b FIX: Map the boolean function argument via Port Specification
                    % This explicitly forces the 'initialize_to_nan' argument to stay hidden as an internal 
                    % block parameter instead of drawing an unwanted left-side port arrow on your canvas face.
                    portSpecs = get_param(c_caller_path, 'FunctionPortSpecification');
                    if ~isempty(portSpecs) && ~isempty(portSpecs.InputArguments)
                        portSpecs.InputArguments(1).Scope = 'Parameter';
                        if strcmp(init_value, 'NaN')
                            set_param(c_caller_path, portSpecs.InputArguments(1).Name, 'true');
                        else
                            set_param(c_caller_path, portSpecs.InputArguments(1).Name, 'false');
                        end
                    end

                    % 3. Explicitly specify the output data type as the bus type
                    set_param(outport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);
                catch
                end
            end
        end

        function uorb_topic(callbackContext)
            % Mask parameter callback - updates dropdown choices when user edits topic parameter.
            %
            % Called whenever the 'uorb_topic' parameter value changes.
            % Refreshes the list of available topics from px4API to reflect current PX4 messages.
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
