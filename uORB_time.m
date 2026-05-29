classdef uORB_time
    methods(Static)
        function MaskInitialization(maskInitContext)
            apiInstance = px4API(); % Enforce constructor validation and sync checks
            
            blockHandle = maskInitContext.BlockHandle;
            blockPath = getfullname(blockHandle);        
            
            sample_time_val = get_param(blockHandle, 'sample_time'); 
            
            c_caller_path = [blockPath '/C_Caller'];
            outport_path = [blockPath '/Out1'];

            set_param(c_caller_path, 'SampleTime', sample_time_val);
            set_param(outport_path, 'SampleTime', sample_time_val);
            
            if strcmp(get_param(bdroot(blockHandle), 'Lock'), 'off')
                try
                    % Lock the C Caller block to your new high-resolution timer function signature
                    set_param(c_caller_path, 'FunctionName', 'read_px4_system_time');
                    set_param(outport_path, 'OutDataTypeStr', 'uint64');
                catch
                end
            end
        end
    end
end
