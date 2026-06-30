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

classdef module < handle
    properties (Access = private)
        name_str    (1,1) string = ""
        groups_list = {}
    end
    
    methods
        function obj = module(mandatory_module_name)
            % 1. Enforce uppercase styling
            raw_name = upper(string(mandatory_module_name));
            
            % 2. Silently swap accidental spaces with underscores to protect YAML syntax
            obj.name_str = strrep(raw_name, " ", "_");
        end
        
        function obj = add_group(obj, group_object)
            if ~isa(group_object, 'px4io.parameter.group')
                error('px4io:parameter:module:add_group', 'Input must be a valid px4io.parameter.group instance.');
            end
            obj.groups_list{end+1} = group_object;
        end
        
        function yaml_str = to_yaml(obj)
            % Prepend the mandatory module_name parameter block following the string conversion fix
            yaml_str = string(sprintf('module_name: %s\nparameters:\n', obj.name_str));
            
            for i = 1:length(obj.groups_list)
                yaml_str = yaml_str + obj.groups_list{i}.to_yaml();
            end
        end
        
        function to_file(obj, absolute_file_path)
            yaml_content = obj.to_yaml();
            
            fid = fopen(absolute_file_path, 'w', 'native', 'UTF-8');
            if fid == -1
                error('px4io:parameter:module:to_file', 'Unable to create or write to file path: %s', absolute_file_path);
            end
            
            try
                fprintf(fid, '%s', yaml_content);
                fclose(fid);
                fprintf('[PX4 Exporter] module.yaml successfully exported to: %s\n', absolute_file_path);
            catch ME
                fclose(fid);
                rethrow(ME);
            end
        end
    end
end
