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
            
            apiInstance = px4io.px4API(); % Enforce constructor validation and sync checks

            blockHandle = maskInitContext.BlockHandle;
            maskObj = maskInitContext.MaskObject; % Grab the mask object wrapper
            blockPath = getfullname(blockHandle);

            % Unpack user selection from the active mask parameters table
            uorb_topic = get_param(blockHandle, 'uorb_topic');
            sample_time_val = get_param(blockHandle, 'sample_time');

            % Escape gracefully if block is freshly placed and unconfigured
            if isempty(uorb_topic) || strcmp(uorb_topic, '<empty>') || isempty(strtrim(uorb_topic))
                variantParam = maskObj.getParameter('uorb_variant');
                if ~isempty(variantParam)
                    variantParam.Visible = 'off';
                end
                return;
            end

            % DYNAMIC VISIBILITY CONTROL & BACKEND POPULATION
            variantParam = maskObj.getParameter('uorb_variant');
            if ~isempty(variantParam)
                variants = apiInstance.getTopicVariants(uorb_topic);
                
                % Ensure the underlying list options are configured properly
                if isempty(variants) || (isscalar(variants) && strcmp(variants{1}, '<empty>'))
                    variants = {uorb_topic};
                end
                variantParam.TypeOptions = variants;

                % ALWAYS POPULATE INTERNALLY: Fallback to base topic if invalid or empty
                currentVariant = get_param(blockHandle, 'uorb_variant');
                if isempty(currentVariant) || strcmp(currentVariant, '<empty>') || ~any(strcmp(currentVariant, variants))
                    set_param(blockHandle, 'uorb_variant', variants{1});
                end

                % Evaluate structural hidden conditions
                isSingleRedundantOption = (isscalar(variants)) && strcmp(variants{1}, uorb_topic);
                
                if isSingleRedundantOption
                    variantParam.Visible = 'off'; % Hide the redundant UI field
                else
                    variantParam.Visible = 'on';  % Show because multiple options or unique names exist
                end
            end

            % Re-fetch target topic now that internal fields are guaranteed to have values
            selectedTopic = get_param(blockHandle, 'uorb_variant');
            if isempty(selectedTopic) || strcmp(selectedTopic, '<empty>')
                selectedTopic = uorb_topic;
            end

            % Determine the base message type for the selected topic or variant.
            baseTopic = apiInstance.getBaseTopicForVariant(selectedTopic);
            bus_name = [baseTopic, '_s'];

            c_caller_path = [blockPath '/C_Caller'];
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);

            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % Dynamically map the C Caller to the return-by-value function name.
                    set_param(c_caller_path, 'FunctionName', ['read_' selectedTopic]);

                    % Explicitly specify the output data type as the bus type.
                    set_param(outport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);
                catch
                end
            end
        end

        function uorb_topic(callbackContext)
            % Mask parameter callback - updates dropdown choices when base topic changes.
            blockHandle = callbackContext.BlockHandle;
            maskObj = Simulink.Mask.get(blockHandle);
            
            % Update base topics dropdown
            choices = strsplit(px4io.px4API.getBaseTopicsDropdownString(), ',');
            paramObj = maskObj.getParameter('uorb_topic');
            if ~isempty(paramObj)
                paramObj.TypeOptions = choices;
            end

            % Update variant dropdown based on selected base topic
            variantParam = maskObj.getParameter('uorb_variant');
            if ~isempty(variantParam)
                apiInstance = px4io.px4API();
                uorb_topic = get_param(blockHandle, 'uorb_topic');
                
                if isempty(uorb_topic) || strcmp(uorb_topic, '<empty>') || isempty(strtrim(uorb_topic))
                    variantParam.Visible = 'off';
                    set_param(blockHandle, 'uorb_variant', '');
                else
                    variants = apiInstance.getTopicVariants(uorb_topic);
                    
                    % Safe structural check: Ensure variants fallback array is valid
                    if isempty(variants) || (isscalar(variants) && strcmp(variants{1}, '<empty>'))
                        variants = {uorb_topic};
                    end
                    variantParam.TypeOptions = variants;
                    
                    % Auto-select first variant or keep current if still valid
                    currentVariant = get_param(blockHandle, 'uorb_variant');
                    if isempty(currentVariant) || strcmp(currentVariant, '<empty>') || ~any(strcmp(currentVariant, variants))
                        set_param(blockHandle, 'uorb_variant', variants{1});
                    end
                    
                    % INTERACTIVE VISIBILITY CONTROL
                    isSingleRedundantOption = (isscalar(variants)) && strcmp(variants{1}, uorb_topic);
                    
                    if isSingleRedundantOption
                        variantParam.Visible = 'off'; 
                    else
                        variantParam.Visible = 'on';  
                    end
                end
            end
        end
    end
end
