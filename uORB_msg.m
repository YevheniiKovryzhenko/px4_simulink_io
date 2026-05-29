classdef uORB_msg
    methods(Static)
        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle 
        %  - MaskObject 
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.        
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
