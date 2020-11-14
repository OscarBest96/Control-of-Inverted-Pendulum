%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 10/02/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate non-linear Arduino suitable simulation of uncontrolled inverted pendulum

close all
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% consider equation
% d2theta/dt2 - a1*dtheta/dt -a2*theta + b0* ndu/dt + b1*u
% this equation captures the dynamics of the inverted pendulum 
% values calculated from inverted pendulum ODE equation
b0 =  1.442;
b1 = 0;
a0 = 0;
a1 = 0.359;
a2 = -14.136;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup time points
dt =  0.0400;
Tfinal = 5;
t = 0:dt:Tfinal;

%LuenburgerObserver 2x2 matricies used for calculation
Aobsv = [0 1; -a2 -a1];
Bobsv = [b0; -a1*b0];
Cobsv = [1 0];
PX= 20*[-1 -1.2];
L= place(Aobsv,Cobsv',PX);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%State space model
A = [0 1 0 0; -a2 -a1 0 0; 0 0 0 0; 0 0 1 0];     
B = [b0; -a1*b0; 1; 0];     
C = [1 0 0 0];
D = 0;

%Gain values for desired pole placement
px = 4* [-0.5, -0.6, -0.7, -0.8];
K = place(A, B, px)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'Example Uncontrolled Inverted Pendulum: 10532799';
disp(titleMessage)

% initialize arrays
xData = [];
yData=[];
tData=[];
kickFlag=[];

% for sub-loop runs
runs = 2;
for kick=1:runs
        
    % for each run randomly perturb intial condition
    x0 = [0; 1 * (rand - 0.5); 0; 0];
    
    % run Euler integration
    [y, t, x] = StateSpaceIntegrator(@CBNLVCPend, a1, a2, b0, C, Aobsv, Bobsv, Cobsv, D, K, L, t, x0);
    
    % get time
    newTime = (kick-1) * t(end) + t;
    
    % just show kick arrow for short time after kick
    frames = length(t);
    kickFlagK = zeros(1,frames);
    if(x0(2) > 0)
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = -abs(x0(2));
    else
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = abs(x0(2));
    end
    
    % concatenate data between runs
    tData = [tData newTime];
    yData = [yData y];
    xData = [xData x];
    kickFlag = [kickFlag kickFlagK];
end

% plot out the state variables
PlotStateVariable2x2(xData, tData, titleMessage);

% for all time point animate the results
figure
range=1;

% cart not moving so set distance to zero
distance = xData(3,:);

% use animate function
AnimatePendulumCart( (yData + pi),  distance, 0.6, tData, range, kickFlag, titleMessage);


