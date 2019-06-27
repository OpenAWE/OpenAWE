conf = awe.model.ampyx_ap2_conf();

conf.tether_length = 330;

% setup wind and environment
conf.wind = struct;
conf.wind.atBaseAltitude = [0;0;0];
conf.wind.baseAltitude   = 6.5;
conf.wind.exponent       = 0.12;
conf.wind.is_constant    = true;

conf.airDensity        = 1.225;
conf.gravNav           = [0;0;9.81];

p = [0;0;-100];
v = [10;0;0];
R = eye(3);
omega = [0;0;0];

x0 = [p;v;reshape(R,9,1);omega];

controller = @awe.simulation.fuzzy_controller;

tspan = 0:0.01:20;
[t,y] = ode45(@(t,x)awe.simulation.dynamics(x,conf,controller), tspan, x0);

alpha = zeros(size(y,1), 1);
airspeed = zeros(size(y,1), 1);
for k=1:size(y, 1)
  v = y(k, 4:6)';
  R = reshape(y(k, 7:15)', 3, 3);
  [airspeed(k),alpha(k),~] = awe.model.aerodynamic_angles(v, R, conf.wind.atBaseAltitude);
end


figure;

subplot(4,2,1);
plot(t, -y(:,3)');
ylabel('height')

subplot(4,2,3);
plot(t, airspeed);
ylabel('airspeed')

subplot(4,2,5);
plot(t, alpha);
ylabel('alpha')

subplot(4,2,7);
plot(t, y(:,17)');
ylabel('pitch rate')

xlabel('time')

subplot(4,2,[2,4,6,8]);
plot(y(:,1)',-y(:,3)')
ylabel('height')
xlabel('distance')


