function data = jumping_analysis(t,a)
% Method for estimating sacral kinematics during counter movement vertical
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
% 1. time - time (s, nx1) 
% 2. a - acceleration (g, nx3) 
%
% Outputs:
% 1. data - structure that inclues:
%           a. kinematic time series 
%               - vertical acceleration
%               - vertical velocity
%               - vertical displacment
%           b. kinematic parameters:
%               - jh - jump height
%               - vp - max vertical acceleration
%               - amax - max vertical velocity
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

%Adjust variable names
a_inertial = a;
fs = 1/mean(diff(t));

%Calculate velocity
v = cumtrapz(t,a);

%Linear drift correction for velocity (zero velocity)
t1 = t(1);
t2 = t(end);
c = [1, t1; 1, t2] \ [v(1,:); v(end,:)];
v_inertial = v - [t.^0, t] * c;

%Calculate displacement
d = cumtrapz(t,v_inertial);

% Assume linear drift in displacement and correct vertical displacement
%Considers: zero net displacement at end, zero velocity at beginning,
%           zero velocity at end
d1 = d(1,:);
dd1 = (d(2,:)-d(1,:))*fs; %v1
d2 = d(end,:);
dd2 = (d(end,:)-d(end-1,:))*fs; %v2
t1 = t(1);
t2 = t(end);

w1 = 1; %d2 = 0
w2 = 1; %v1 = 0
w3 = 1; %v2 = 0

X = [2*w1, 2*t2*w1, -1;
    2*t2*w1, (2*w1*t2^2+2*w2+2*w3), -t1;
    1, t1, 0];
Y = [2*w1*d2; 2*w1*t2*d2 + 2*w2*dd1 + 2*w3*dd2; d1];
c = X \ Y;

d_inertial = d - [t.^0, t] * c(1:2,:);

raw.d = d_inertial;
raw.v = v_inertial;
raw.a = a_inertial;
raw.time = t;

[jh,i_ff] = max(d_inertial(:,3));
[vmax,i_vmax] = max(v_inertial(1:i_ff,3));
[amax,~] = max(a_inertial(1:i_ff,3));
data = jumping_metrics(i_vmax,raw);

data.jh = jh;
data.vp = vmax;
data.amax = amax;
data.raw = raw;
end
