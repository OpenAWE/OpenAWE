% basic consistency checks for the linear acceleration model

p = [0;0;0];
v = [10;0;0];
R = eye(3);
omega = [0;0;0];
lambda = 5000;

conf = awe.model.ampyx_ap2_conf();

conf.wind = struct;
conf.wind.atBaseAltitude = [-10;0;0];
conf.wind.baseAltitude   = 6.5;
conf.wind.exponent       = 0.12;
conf.wind.is_constant    = true;

conf.airDensity = 1.22;
conf.gravNav = [0;0;9.81];
coefficients = conf.coefficients;
wingSpan = conf.wingSpan;
wingArea = conf.wingArea;
chord = conf.chord;

awe.model.linear_acceleration(p,v,R,omega,lambda,conf);