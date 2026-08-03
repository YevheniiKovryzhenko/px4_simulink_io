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

classdef float < px4io.parameter.base
    properties (Access = private)
        decimal_places = 4
        increment_step = NaN
    end
    
    methods
        function obj = float(default_value)
            obj@px4io.parameter.base(single(default_value));
        end
        
        function obj = set_decimal(obj, val),   obj.decimal_places = val; end
        function obj = set_increment(obj, val), obj.increment_step = val; end
        
        function yaml_str = to_yaml(obj)
            fmt = string(sprintf('%%.%df', obj.decimal_places));
            yaml_str = obj.generate_common_yaml('float', fmt);
            
            yaml_str = yaml_str + sprintf('              decimal: %d\n', obj.decimal_places);
            if ~isnan(obj.increment_step)
                yaml_str = yaml_str + sprintf(['              increment: ', char(fmt), '\n'], obj.increment_step);
            end
        end
    end
end