classdef uORB_write
    methods(Static)
        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle 
        %  - MaskObject 
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)
            apiInstance = px4API(); % Enforce constructor validation and sync checks
            
            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);
            
            % Unpack user selection from the active mask parameters table
            uorb_topic = get_param(blockHandle, 'uorb_topic');
            sample_time_val = get_param(blockHandle, 'sample_time'); % Read your mask parameter
            
            % Escape gracefully if block is freshly placed and unconfigured
            if isempty(uorb_topic) || strcmp(uorb_topic, '<empty>') || isempty(strtrim(uorb_topic))
                return;
            end
            
            % Native PX4 snake_case naming - no conversions needed
            bus_name = [uorb_topic, '_s'];
            
            c_caller_path = [blockPath '/C_Caller'];
            inport_path = [blockPath '/In1']; % FIXED: Targets the input boundary block

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(inport_path, 'SampleTime', sample_time_val);
            
            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % FIXED: Maps the C Caller to your pass-by-value function name 'write_'
                    set_param(c_caller_path, 'FunctionName', ['write_' uorb_topic]);
                    
                    % Explicitly specify the input data type as the bus type
                    set_param(inport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);
                catch
                end
            end
        end

        % Callback that handles manual user edits in the dropdown dialog GUI
        function uorb_topic(callbackContext)
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