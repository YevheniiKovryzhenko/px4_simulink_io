classdef uORB_read
    methods(Static)
        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle 
        %  - MaskObject 
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)
            apiInstance = px4API(); % Enforce constructor validation and sync checks
            
            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);
            maskObj = maskInitContext.MaskObject;
            
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
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);
            
            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % FIXED: Dynamically map the C Caller to your return-by-value function name.
                    % Simulink handles ports, definitions, and pins automatically now.
                    set_param(c_caller_path, 'FunctionName', ['read_' uorb_topic]);
                    
                    % Explicitly specify the output data type as the bus type
                    % This allows Simulink to properly resolve the structure definition
                    set_param(outport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);

                    % Programmatically inject a command that draws the current topic string 
                    % directly onto the face of the white subsystem canvas rectangle block.
                    % \\n adds a clean line break for visual styling.
                    maskObj.Display = sprintf('disp(''%s'');', uorb_topic);
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
