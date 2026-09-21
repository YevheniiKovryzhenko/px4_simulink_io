% /****************************************************************************
%  *
%  *    Copyright (C) 2026  Yevhenii Kovryzhenko. All rights reserved.
%  *
%  *    This program is free software: you can redistribute it and/or modify
%  *    it under the terms of the GNU Affero General Public License as published by
%  *    the Free Software Foundation, either version 3 of the License, or
%  *    (at your option) any later version.
%  *
%  *    This program is distributed in the hope that it will be useful,
%  *    but WITHOUT ANY WARRANTY; without even the implied warranty of
%  *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  *    GNU Affero General Public License Version 3 for more details.
%  *
%  *    You should have received a copy of the
%  *    GNU Affero General Public License Version 3
%  *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
%  *
%  *    1. Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions, and the following disclaimer.
%  *    2. Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions, and the following disclaimer in
%  *       the documentation and/or other materials provided with the
%  *       distribution.
%  *    3. No ownership or credit shall be claimed by anyone not mentioned in
%  *       the above copyright statement.
%  *    4. Any redistribution or public use of this software, in whole or in part,
%  *       whether standalone or as part of a different project, must remain
%  *       under the terms of the GNU Affero General Public License Version 3,
%  *       and all distributions in binary form must be accompanied by a copy of
%  *       the source code, as stated in the GNU Affero General Public License.
%  *
%  ****************************************************************************/

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

classdef uORB_msg
    methods(Static)
        % maskInitContext properties available:
        %  - BlockHandle: Handle to the mask (this block)
        %  - MaskObject: Simulink mask object
        %  - MaskWorkspace: Access to mask parameter values
        
        function MaskInitialization(maskInitContext)
            apiInstance = px4io.px4API.getInstance();

            blockHandle = maskInitContext.BlockHandle;
            maskObj = maskInitContext.MaskObject; % Grab the mask object wrapper
            blockPath = getfullname(blockHandle);

            % Unpack user selections from the active mask parameters table
            uorb_topic = get_param(blockHandle, 'uorb_topic');
            sample_time_val = get_param(blockHandle, 'sample_time'); 
            init_value = get_param(blockHandle, 'init_value'); 

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
                % 1. Dynamically map the C Caller to your return-by-value initialization function name
                set_param(c_caller_path, 'FunctionName', ['init_' selectedTopic]);

                % 2. Map the boolean function argument via Port Specification
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
                apiInstance = px4io.px4API.getInstance();
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
