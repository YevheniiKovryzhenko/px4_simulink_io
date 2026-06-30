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

classdef unit
    properties
        Value (1,1) string
    end
    
    methods
        function obj = unit(val)
            obj.Value = string(val);
        end
        
        % This lets us convert the enum instance to a raw char array easily
        function c = char(obj)
            c = char(obj.Value);
        end
    end
    
    enumeration
        % Kinematics & Spatial
        M           ("m")       
        M_S         ("m/s")     
        M_S2        ("m/s^2")   
        M_S3        ("m/s^3")   
        MM          ("mm")      
        CM          ("cm")      
        KM          ("km")      
        KM_H        ("km/h")    
        
        % Rotation & Dynamics
        RAD         ("rad")     
        RAD_S       ("rad/s")   
        RAD_S2      ("rad/s^2") 
        DEG         ("deg")     
        DEG_S       ("deg/s")   
        KG          ("kg")      
        G           ("g")       
        N           ("N")       
        NM          ("Nm")      
        KG_M2       ("kg m^2")  
        
        % System, Time & Electrical
        S           ("s")       
        MS          ("ms")      
        US          ("us")      
        HZ          ("Hz")      
        V           ("V")       
        A           ("A")       
        MAH         ("mAh")     
        CELSIUS     ("C")       
        PERCENT     ("%")       
        NORM        ("norm")    
    end
end
