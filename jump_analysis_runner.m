%% Vertical Jumping Analysis Runner
% This script provides example implementation of code for analyzing 
% countermovement vertical jumping biomechanics  
% Written by Ryan S. McGinnis - ryan.mcginis14@gmail.com - July 9, 2016

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


%% Load data file and import orientation functions
addpath('C:\Users\ryanmcg\Documents\Repos\IMU-orientation'); % add location of IMU orientaiton repo to path
load('example_data.mat');
t = data.t;
a = data.a; %m/s^2
w = data.w; %rad/s

% Manage memory
clear data


%% Select important time points from data file
%1. Start of still section immediately before jump
%2. End of still section immediately before jump
%3. Still point immediately after jump
figure;
plot(t,sqrt(sum(a.^2,2)));
xlabel('Time (s)'); ylabel('Acceleration Magnitude (m/s^2)');
set(gcf,'name','zoom to jump and hit a key')
pause;
set(gcf,'name','select still section before and still point after jump');
[xs,~] = ginput(3);
close(gcf);


%% Trucate data to only include jump and define orientation 
a = a(t>xs(1) & t<xs(3),:); % g
w = w(t>xs(1) & t<xs(3),:);
t_j = t(t>xs(1) & t<xs(3));
ind = t_j > xs(1) & t_j < xs(2);
q = get_orientation_compfilter_quaternion(t_j, a, w*180/pi, ind);


%% Define variables needed to calculate jump kinematic trajectories
% Find still sections during jump
amag = sqrt(sum(a.^2,2));
wmag = sqrt(sum(w.^2,2));
g_mag = mean(amag(ind));
a_noise = std(amag(ind));
w0_mag = mean(wmag(ind));
w_noise = std(wmag(ind));

a_ub = 3*a_noise + g_mag;
a_lb = -3*a_noise + g_mag;
w_ub = w0_mag + 3*w_noise;
jump_still = amag > a_lb & amag < a_ub & wmag < w_ub;

% Resolve accelerometer data in world-frame and remove gravity
a_j = quaternRot(q,a) - (t_j.^0)*[0,0,g_mag];


%% Select potential beginning and ending points for the jumping motion
figure;
hold on;
plot(t_j,amag);
plot(t_j(jump_still),amag(jump_still),'.k');
xlabel('time'); ylabel('A_{mag}');
set(gcf,'name','hit any key and select approximate start and end of jump');
pause;
[xs,~] = ginput(2);
close(gcf);

%Identify still pt. closest to selected start and end of jump
t_jb = t_j(t_j<xs(1) & jump_still);
if ~isempty(t_jb)
    t1 = t_jb(end);
    if abs(t1-xs(1))>0.25
        t1=xs(1);
    end
else
    t1=xs(1);
end

t_ja = t_j(t_j>xs(2) & jump_still);
if ~isempty(t_ja)
    t2 = t_ja(1);
    if abs(t2-xs(2))>0.25
        t2=xs(2);
    end
else
    t2=xs(2);
end
t_ji = t_j(t_j>=t1 & t_j<=t2);
a_ji = a_j(t_j>=t1 & t_j<=t2,:);


%% Calculate jump kinematic trajectories and performance metrics
data = jumping_analysis(t_ji,a_ji);


%% Plot kinematic trajectories
figure;
grid on; hold on;
plot(data.raw.time,data.raw.a(:,3),'k','linewidth',2);
xlabel('Time (s)'); ylabel('Vertical Acceleration (m/s^2)');

figure;
grid on; hold on;
plot(data.raw.time,data.raw.v(:,3),'k','linewidth',2);
xlabel('Time (s)'); ylabel('Vertical Velocity (m/s)');

figure;
grid on; hold on;
plot(data.raw.time,data.raw.d(:,3),'k','linewidth',2);
xlabel('Time (s)'); ylabel('Vertical Displacement (m)');

