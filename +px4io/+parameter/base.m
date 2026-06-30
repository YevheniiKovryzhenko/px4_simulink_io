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

classdef (Abstract) base
    properties (Access = private)
        name            (1,1) string = ""
        short_desc      (1,1) string = ""
        long_desc       (1,1) string = ""
        default_val
        min_val         = NaN
        max_val         = NaN
        num_instances   (1,1) int32 = 1
        instance_start  (1,1) int32 = 0
        param_unit      = px4io.parameter.unit.empty()
    end
    
    properties (Access = private, Constant)
        MAX_LEN         (1,1) int32 = 16
    end
    
    methods
        function obj = base(default_value)
            obj.default_val = default_value;
        end
        
        function obj = set_name(obj, val)
            % 1. Convert to string array and force uppercase
            raw_name = upper(string(val));
            
            % 2. Silently replace all spaces with underscores to protect PX4 syntax
            obj.name = strrep(raw_name, " ", "_");
        end

        
        function obj = set_description(obj, short_txt, long_txt)
            obj.short_desc = string(short_txt);
            if nargin > 2
                obj.long_desc = string(long_txt);
            end
        end

        function obj = set_unit(obj, unit_enum)
            if ~isa(unit_enum, 'px4io.parameter.unit')
                error('px4io:parameter:base:set_unit', 'Unit must be a valid entry from the px4io.parameter.unit enumeration.');
            end
            obj.param_unit = unit_enum;
        end
        
        function obj = set_min(obj, val),        obj.min_val = val; end
        function obj = set_max(obj, val),        obj.max_val = val; end
        function obj = set_instances(obj, count, start_idx)
            obj.num_instances = int32(count);
            obj.instance_start = int32(start_idx);
        end
    end
    
    methods (Access = protected)
        function txt = generate_common_yaml(obj, type_str, float_fmt_override)
            % Assemble output tokens and validate the final evaluated string size
            if obj.num_instances > 1
                final_yaml_token = obj.name + "${i}";
                
                % Calculate final name length by replacing the 4-character "${i}" with a 1-character index digit
                actual_final_name = strrep(final_yaml_token, "${i}", "1");
                token_len = strlength(actual_final_name);
                
                if token_len > obj.MAX_LEN
                    fprintf('[PX4 Naming Warning] Vector parameter name "%s" exceeds size boundaries (%d/%d characters).\n', ...
                        actual_final_name, token_len, obj.MAX_LEN);
                end
                txt = string(sprintf('          %s:\n', final_yaml_token));
            else
                token_len = strlength(obj.name);
                if token_len > obj.MAX_LEN
                    fprintf('[PX4 Naming Warning] Scalar parameter name "%s" exceeds size boundaries (%d/%d characters).\n', ...
                        obj.name, token_len, obj.MAX_LEN);
                end
                txt = string(sprintf('          %s:\n', obj.name));
            end
            
            % Omit description blocks entirely if both short and long are empty
            if obj.short_desc ~= "" || obj.long_desc ~= ""
                txt = txt + string(sprintf('              description:\n'));
                if obj.short_desc ~= ""
                    txt = txt + string(sprintf('                  short: %s\n', obj.short_desc));
                end
                if obj.long_desc ~= ""
                    txt = txt + string(sprintf('                  long: |\n'));
                    lines = split(obj.long_desc, newline);
                    for i = 1:length(lines)
                        txt = txt + string(sprintf('                      %s\n', lines{i}));
                    end
                end
            end
            
            txt = txt + string(sprintf('              type: %s\n', type_str));

            if ~isempty(obj.param_unit)
                txt = txt + string(sprintf('              unit: %s\n', char(obj.param_unit)));
            end
            
            if nargin < 3, float_fmt_override = "%.4f"; end
            
            if obj.num_instances > 1
                txt = txt + string(sprintf('              num_instances: %d\n', obj.num_instances));
                txt = txt + string(sprintf('              instance_start: %d\n', obj.instance_start));
                txt = txt + string(sprintf('              default: ['));
                for idx = 1:length(obj.default_val)
                    if isfloat(obj.default_val)
                        txt = txt + string(sprintf(float_fmt_override, obj.default_val(idx)));
                    else
                        txt = txt + string(sprintf('%d', obj.default_val(idx)));
                    end
                    if idx < length(obj.default_val), txt = txt + string(sprintf(', ')); end
                end
                txt = txt + string(sprintf(']\n'));
            else
                if isfloat(obj.default_val)
                    txt = txt + string(sprintf(['              default: ', char(float_fmt_override), '\n'], obj.default_val));
                else
                    txt = txt + string(sprintf('              default: %d\n', obj.default_val));
                end
            end
            
            if ~isnan(obj.min_val)
                if isfloat(obj.min_val), txt = txt + string(sprintf(['              min: ', char(float_fmt_override), '\n'], obj.min_val));
                else,                    txt = txt + string(sprintf('              min: %d\n', obj.min_val)); end
            end
            if ~isnan(obj.max_val)
                if isfloat(obj.max_val), txt = txt + string(sprintf(['              max: ', char(float_fmt_override), '\n'], obj.max_val));
                else,                    txt = txt + string(sprintf('              max: %d\n', obj.max_val)); end
            end
        end
    end
    
    methods (Abstract)
        yaml_str = to_yaml(obj)
    end
end
