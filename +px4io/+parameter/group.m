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

classdef group < handle
    properties (Access = private)
        group_name    (1,1) string = ""
        param_items   = {}
    end
    
    methods
        function obj = group(name)
            obj.group_name = string(name);
        end
        
        function obj = add(obj, parameter_object)
            % Check relative local namespace types safely
            if ~isa(parameter_object, 'px4io.parameter.base') && ...
               ~isa(parameter_object, 'px4io.parameter.matrix')
                error('px4io:parameter:group:add', 'Only valid px4io parameters or matrices can be added.');
            end
            obj.param_items{end+1} = parameter_object;
        end
        
        function yaml_str = to_yaml(obj)
            if isempty(obj.param_items)
                yaml_str = "";
                return;
            end
            
            yaml_str = sprintf('    - group: %s\n      definitions:\n', obj.group_name);
            for i = 1:length(obj.param_items)
                yaml_str = yaml_str + obj.param_items{i}.to_yaml();
            end
        end
    end
end
