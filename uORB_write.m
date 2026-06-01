% uORB_write - Mask initialization for uORB topic writer blocks
%
% This mask class configures the mask initialization function for a Simulink subsystem
% that writes (publishes) a single uORB topic structure.
%
% Block interface:
%   Parameters:
%     - uorb_topic (string): PX4 topic name (snake_case, auto-populated dropdown)
%     - sample_time (string): Simulink sample time (e.g., '-1' for inherited)
%
%   Input port (In1):
%     - Type: Bus (named <uorb_topic>_s)
%     - Input to write_<uorb_topic>() C function
%
% Functionality:
%   1. Maps block to write_<uorb_topic>() C function (pass-by-value)
%   2. Configures input port data type to match topic structure
%   3. Updates dropdown choices when topic parameter is edited
%
% Generated C code:
%   void write_<topic>(struct <topic>_s in)
%     - Publishes struct to uORB
%     - Advertise publisher on first call, publish on subsequent calls
%
% Usage:
%   1. Place block from Simulink library
%   2. Select uorb_topic from dropdown
%   3. Block automatically configures C function call
%   4. Connect output from blocks producing same topic type
%   5. Sample time controls uORB publish frequency

classdef uORB_write
    methods(Static)
        % maskInitContext properties available:
        %  - BlockHandle: Handle to the mask (this block)
        %  - MaskObject: Simulink mask object
        %  - MaskWorkspace: Access to mask parameter values
        
        function MaskInitialization(maskInitContext)
            % Primary mask initialization function.
            %
            % Configures the C Caller block to call write_<uorb_topic>() function.
            % Sets the input port data type to the corresponding bus structure.
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
            inport_path = [blockPath '/In1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(inport_path, 'SampleTime', sample_time_val);
            
            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % Maps the C Caller to the pass-by-value function name 'write_'.
                    set_param(c_caller_path, 'FunctionName', ['write_' uorb_topic]);
                    
                    % Explicitly specify the input data type as the bus type.
                    set_param(inport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);
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