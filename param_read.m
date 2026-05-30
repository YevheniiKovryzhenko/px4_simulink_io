classdef param_read
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
            param_type = param_read.getParamDatatype(blockHandle);
            param_type = param_read.normalizeParamDatatype(param_type);

            if isempty(param_name) || strcmp(param_name, '<empty>') || isempty(strtrim(param_name))
                return;
            end

            if strcmp(param_type, 'int32')
                function_name = 'read_px4_param_int32';
                out_data_type = 'int32';
            else
                function_name = 'read_px4_param_float';
                out_data_type = 'single';
            end

            c_caller_path = [blockPath '/C_Caller'];
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);

            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    set_param(c_caller_path, 'FunctionName', function_name);

                    set_param(outport_path, 'OutDataTypeStr', out_data_type);
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
