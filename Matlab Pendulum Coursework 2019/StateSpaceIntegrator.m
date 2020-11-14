function [y, tout, xout] = StateSpaceIntegrator(VCPendDotCB, a1, a2, b0, C, Aobsv, Bobsv, Cobsv, D, K, L, t, x0)
% non-linear model state space feedback velocity control using C-language compatible formulation
% performs integration by Euler's method
% VCPendDotCB is the callback function to compute xDot 
% a1, a2, b0 are coedfficients  
% C, D ame the state space model output matrices
% K id the state feedback control ggain - unused here
% t is a vector of time samples
% x0 is the initial state vector
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

% get signal length
len = length(t) -1;

% init output
y = zeros(1,len);
xout = zeros(4,len);

% record the initial state
xout(:, 1) = x0;
x = x0;
xHat = x(1:2);


      
% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
    
    %record time
    tout(idx) = t(idx);

    %get the duration between updates
    h = t(idx+1) - t(idx);
    
    %control
    %estimated states xHat(1) & xHat(2) used instead of x(1)& x(2)
    u = -K(1)* xHat(1) -K(2) * xHat(2) -K(3) * x(3) -K(4) * x(4);
    
    %real theta output
    y(idx) = C*x
   
    %calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(a1, a2, b0, x, u);
    
    %update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);
    x(3) = x(3) + h * xDot(3);  % positional state
    x(4) = x(4) + h * xDot(4);  % integral action
      
    %record the state
    xout(:, idx) = x;
      
    
    %LUENBURGER OBSERVER
    %output error correction term
    Ycorr = L * (y(idx) -Cobsv * xHat);
    
    %state estimate
    xHatdot = Aobsv * xHat + Bobsv * u + Ycorr;
    
    %update using euler
    xHat(1) = xHat(1) + h * xHatdot(1);
    xHat(2) = xHat(2) + h * xHatdot(2);
   
   % calculate output from theta and thetaDot state
   % y(idx)= C(1) * x(1) + C(2) * x(2) + D(1) * u;
    
end



