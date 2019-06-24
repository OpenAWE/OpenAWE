function power_pointconstraints(ch, k, K, x, params)
% consistency conditions
ic = PowerOptimizationOCP.system.getInitialConditions(state0.value,params.value);
self.addBoundaryCondition(ic,'==',0);

%
positionNav = state0.get('positionNav');
velocityNav = state0.get('velocityNav');
rotBodyToNav = state0.get('rotBodyToNav');
bodyAngularRate = state0.get('bodyAngularRate');


positionEnd = stateF.get('positionNav');
self.addBoundaryCondition(positionNav,'==',positionEnd);

velocityEnd = stateF.get('velocityNav');
self.addBoundaryCondition(velocityNav,'==',velocityEnd);

bodyAngularRateEnd = stateF.get('bodyAngularRate');
self.addBoundaryCondition(bodyAngularRate,'==',bodyAngularRateEnd);

% periodic rotation
rotBodyToNavEnd = stateF.get('rotBodyToNav');
[yawEnd,pitchEnd,rollEnd] = GetYawPitchRoll(rotBodyToNavEnd);
[yawStart,pitchStart,rollStart] = GetYawPitchRoll(rotBodyToNav);

self.addBoundaryCondition(yawStart,'==',yawEnd);
self.addBoundaryCondition(pitchStart,'==',pitchEnd);
self.addBoundaryCondition(rollStart,'==',rollEnd);
