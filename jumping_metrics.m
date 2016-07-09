function data = jumping_metrics(max_vi,raw)
% Method for parameterizing sacral kinematics during counter movement vertical
% jump.  
%
% Copyright (C) 2016  Ryan S. McGinnis
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% Method validated in:
% McGinnis, Ryan S., et al. "Quantifying the effects of load carriage and 
% fatigue under load on sacral kinematics during countermovement vertical 
% jump with IMU-based method." Sports Engineering 1.19 (2016): 21-34.
%
% Inputs:
% 1. max_vi - index of maximum vertical velocity 
% 2. raw - structure of time series representing sacral kinematics
%
% Outputs:
% 1. data - structure that inclues kinematic parameters:
%               - vssc - vel range max upward to max downward
%               - vc - max downward vel
%               - tssc - time from max downward to max upward vel
%               - tp - time from stop to max upward vel
%               - tc2 - time from max downward vel to stop
%               - tc1 - time to max downward vel
%               - tj - duration of jump - start of countermovement to max upward vel
%               - ap - avg accel during upward movement
%               - ac2 - avg accel from max downward to stop
%               - ac1 - avg accel from start of countermovement to max downward
%               - dc - displacement at bottom of countermovement

%Extract data up to time of vmax
d = raw.d(1:max_vi,3);
v = raw.v(1:max_vi,3);
a = raw.a(1:max_vi,3);
t = raw.time(1:max_vi);

[~,min_vi] = min(v);
[~,max_vi0] = max(v(1:min_vi));
if v(max_vi0)>0
    [~,zero_vi0] = min(abs(v(max_vi0:min_vi))); zero_vi0 = zero_vi0+max_vi0;
else
    zero_vi0 = 1;
end
[~,zero_vi] = min(abs(v(min_vi:end))); zero_vi = zero_vi+min_vi;

data.vssc = v(max_vi) - v(min_vi); %vel range max upward to max downward
data.vc = v(min_vi); %max downward vel
data.tssc = t(max_vi) - t(min_vi); %time from max downward to max upward vel
data.tp = t(max_vi) - t(zero_vi); %time from stop to max upward vel
data.tc2 = t(zero_vi) - t(min_vi); %time from max downward vel to stop
data.tc1 = t(min_vi) - t(zero_vi0); %time to max downward vel
data.tj = t(max_vi) - t(zero_vi0); %duration of jump - start of countermovement to max upward vel
data.ap = mean(a(zero_vi:max_vi)); %avg accel during upward movement
data.ac2 = mean(a(min_vi:zero_vi)); %avg accel from max downward to stop
data.ac1 = mean(a(zero_vi0:min_vi)); %avg accel from start of countermovement to max downward
data.dc = d(zero_vi); %displacement at bottom of countermovement
end