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

classdef matrix
    properties (Access = private)
        base_name       (1,1) string = ""
        short_desc      (1,1) string = ""
        long_desc       (1,1) string = ""
        matrix_data     
        is_floating     (1,1) logical = true
        decimal_places  (1,1) int32 = 4
        param_unit      = px4io.parameter.unit.empty()
        param_objects   = {}
    end
    
    methods
        function obj = matrix(input_matrix)
            if ~ismatrix(input_matrix) || isempty(input_matrix)
                error('px4io:parameter:matrix', 'Input must be a non-empty 2D matrix.');
            end
            
            obj.is_floating = isfloat(input_matrix) && ~isinteger(input_matrix);
            if obj.is_floating
                obj.matrix_data = single(input_matrix);
            else
                obj.matrix_data = int32(input_matrix);
            end
            
            [rows, cols] = size(obj.matrix_data);
            
            if cols >= rows
                for r = 1:rows
                    row_data = obj.matrix_data(r, :);
                    if obj.is_floating, p = px4io.parameter.float(row_data);
                    else,               p = px4io.parameter.int(row_data); end
                    p = p.set_instances(cols, 1); 
                    obj.param_objects{end+1} = struct('obj', p, 'suffix', "_R" + r);
                end
            else
                for c = 1:cols
                    col_data = obj.matrix_data(:, c)'; 
                    if obj.is_floating, p = px4io.parameter.float(col_data);
                    else,               p = px4io.parameter.int(col_data); end
                    p = p.set_instances(rows, 1);
                    obj.param_objects{end+1} = struct('obj', p, 'suffix', "_C" + c);
                end
            end
        end
        
        function obj = set_name(obj, val)
            obj.base_name = upper(string(val));
        end
        
        function obj = set_description(obj, short_txt, long_txt)
            obj.short_desc = string(short_txt);
            if nargin > 2
                obj.long_desc = string(long_txt);
            end
        end
        
        function obj = set_decimal(obj, val)
            obj.decimal_places = int32(val);
        end

        function obj = set_unit(obj, unit_enum)
            if ~isa(unit_enum, 'px4io.parameter.unit')
                error('px4io:parameter:matrix:set_unit', 'Unit must be a valid entry from the px4io.parameter.unit enumeration.');
            end
            obj.param_unit = unit_enum;
        end
        
        function yaml_str = to_yaml(obj)
            yaml_str = "";
            for i = 1:length(obj.param_objects)
                sub_p = obj.param_objects{i}.obj;
                sub_name = obj.base_name + obj.param_objects{i}.suffix + "_";
                
                % Append suffix only if a short description was actually assigned
                if obj.short_desc ~= ""
                    sub_short = obj.short_desc + " (Axis/Index " + obj.param_objects{i}.suffix + ")";
                else
                    sub_short = "";
                end
                
                sub_p = sub_p.set_name(sub_name) ...
                             .set_description(sub_short, obj.long_desc);
                         
                if obj.is_floating
                    sub_p = sub_p.set_decimal(obj.decimal_places);
                end

                if ~isempty(obj.param_unit)
                    sub_p = sub_p.set_unit(obj.param_unit);
                end
                
                yaml_str = yaml_str + sub_p.to_yaml();
            end
        end
    end
end