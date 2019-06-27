% basic consistency checks for the aerodynamic forces model

% flying in normal condition should create positive lift and drag forces
p = [0;0;0];
v = [10;0;0];
R = eye(3);
omega = [0;0;0];

w = [0;0;0]; % wind

conf = awe.model.ampyx_ap2_conf();

airDensity = 1.22;

coefficients = conf.coefficients;
wingSpan = conf.wingSpan;
wingArea = conf.wingArea;
chord = conf.chord;

[airspeed,alpha,beta] = awe.model.aerodynamic_angles(v,R,w);

f1 = awe.model.aerodynamic_forces(alpha, ...
                                 beta, ...
                                 airspeed, ...
                                 omega, ...
                                 coefficients, ...
                                 airDensity, ...
                                 wingSpan, ...
                                 wingArea, ...
                                 chord);
assert(f1(1) < 0); % positive drag force, -x direction  
assert(f1(3) < 0); % positive lift force, -z direction

% increased airspeed should result in increase drag and lift
f2 = awe.model.aerodynamic_forces( alpha, ...
                                   beta, ...
                                   airspeed+1, ...
                                   omega, ...
                                   coefficients, ...
                                   airDensity, ...
                                   wingSpan, ...
                                   wingArea, ...
                                   chord);
assert(abs(f2(1)) > abs(f1(1)));
assert(abs(f2(3)) > abs(f1(3)));

% increased angle of attack should result in increase drag and lift
f3 = awe.model.aerodynamic_forces( alpha+0.1, ...
                                   beta, ...
                                   airspeed, ...
                                   omega, ...
                                   coefficients, ...
                                   airDensity, ...
                                   wingSpan, ...
                                   wingArea, ...
                                   chord);
assert(abs(f3(1)) > abs(f1(1)));
assert(abs(f3(3)) > abs(f1(3)));

