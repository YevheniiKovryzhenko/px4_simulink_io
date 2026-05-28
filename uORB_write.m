classdef uORB_write

    methods(Static)

        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle 
        %  - MaskObject 
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)
            apiInstance = px4API();
            
            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);
            uorb_topic = get_param(blockHandle, 'uorb_topic');
            
            if isempty(uorb_topic)
                error('[px4API:Error] uORB Write Block at "%s" is missing a topic selection. Please open the mask dialog and choose a valid topic.', blockPath);
            end
            
            % Native PX4 snake_case naming - no conversions needed
            bus_name = [uorb_topic, '_s'];
            
            c_caller_path = [blockPath '/C_Caller'];
            inport_path = [blockPath '/In1'];
            
            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % Direct native snake_case function and structure naming
                    set_param(c_caller_path, 'FunctionName', ['write_' uorb_topic]);
                    set_param(inport_path, 'OutDataTypeStr', ['Bus: ' bus_name]);
                catch
                end
            end
        end

        % Use the code browser on the left to add the callbacks.
        function uorb_topic(callbackContext)
            blockHandle = callbackContext.BlockHandle;
            
            % Query the list using your static method on your central class
            choices = strsplit(px4API.getTopicDropdownString(), ',');
            
            % Push choices string straight into the UI dropdown list parameter
            set_param(blockHandle, 'TypeOptions_uorb_topic', choices);
        end
    end
end