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

classdef enum < px4io.parameter.int
    properties (Access = private)
        enum_meta_values = struct('code', {}, 'label', {})
    end
    
    methods
        function obj = enum(enum_instance)
            % 1. Extract the underlying integer code for the default setting
            default_int = int32(enum_instance);
            obj@px4io.parameter.int(default_int);
            
            % 2. Introspect the Enum class metadata automatically
            mc = meta.class.fromName(class(enum_instance));
            if isempty(mc) || ~mc.Enumeration
                error('px4io:parameter:enum', 'The provided value must be a valid MATLAB class enumeration.');
            end
            
            % 3. Extract and cache all defined enum items and numeric keys
            enum_list = mc.EnumerationMemberList;
            for i = 1:length(enum_list)
                item_name = string(enum_list(i).Name);
                % Evaluate the enumeration constant name to fetch its assigned integer value
                item_code = int32(eval(class(enum_instance) + "." + item_name));
                
                obj.enum_meta_values(i).code = item_code;
                obj.enum_meta_values(i).label = item_name;
            end
        end
        
        function yaml_str = to_yaml(obj)
            % Generate base structure definitions from the parent integer layout
            yaml_str = obj.generate_common_yaml('int32');
            
            % Append the dynamically parsed enumeration menu table block
            if ~isempty(obj.enum_meta_values)
                yaml_str = yaml_str + sprintf('              values:\n');
                for i = 1:length(obj.enum_meta_values)
                    yaml_str = yaml_str + sprintf('                  %d: %s\n', ...
                        obj.enum_meta_values(i).code, obj.enum_meta_values(i).label);
                end
            end
        end
    end
end