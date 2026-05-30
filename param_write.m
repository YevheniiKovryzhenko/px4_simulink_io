classdef param_write
    methods(Static)
        % Following properties of 'maskInitContext' are available to use:
        %  - BlockHandle
        %  - MaskObject
        %  - MaskWorkspace: Use get/set APIs to work with mask workspace.
        function MaskInitialization(maskInitContext)
            px4API();

            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);

            param_name = get_param(blockHandle, 'param_name');
            sample_time_val = get_param(blockHandle, 'sample_time');
            param_type = param_write.getParamDatatype(blockHandle);
            param_type = param_write.normalizeParamDatatype(param_type);

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
            paramType = lower(strtrim(string(paramType)));
            if paramType == "single" || paramType == "float32"
                paramType = 'float';
            else
                paramType = 'int32';
            end
        end
    end
end
