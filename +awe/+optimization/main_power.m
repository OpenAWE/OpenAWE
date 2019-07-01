N = 50;
T = 44;

conf = awe.model.ampyx_ap2_conf();

conf.tether_length = 330;

% setup wind and environment
conf.wind = struct;
conf.wind.atBaseAltitude = [-10;0;0];
conf.wind.baseAltitude   = 6.5;
conf.wind.exponent       = 0.12;
conf.wind.is_constant    = true;

conf.airDensity        = 1.225;
conf.gravNav           = [0;0;9.81];

conf.w_referenceTracking = 1;
conf.w_bodyAngularAccel = 1e2;
conf.w_ddl = 1e-2;
conf.w_beta = 1e2;
conf.w_integratedWork = 1e-3;

conf.MAX_AIRSPEED = 32;
conf.MIN_AIRSPEED = 13;
conf.MAX_ALPHA = 0.16;
conf.MIN_ALPHA = -0.1;
conf.MAX_BETA = 0.3;
conf.MIN_BETA = -0.3;
conf.MAX_TENSION = 5000;
conf.MIN_TENSION = 10;

[ref_p,ref_v,ref_R] = awe.optimization.reference_path(N, T);

solver = ocl.Solver(T, @awe.optimization.variables, ...
                    @(h,x,z,u,p) awe.optimization.dynamics(h, x, z, u, conf), ...
                    @(h,x,z,u,p) awe.optimization.path_cost(h, x, u, conf), ...
                    @(h,k,K,x,p) awe.optimization.point_cost(h, k, K, x, p, conf, ref_p), ...
                    @(h,k,K,x,p) awe.optimization.point_constraints(h,k,K,x,conf), ...
                    'N', N);

% solver.setBounds('lambda', 1, 100000);

solver.setBounds('p', [-10000; -10000; -10000], [10000; 10000; -100]);
solver.setBounds('v', -60, 60);
solver.setBounds('R', -1.1, 1.1);
solver.setBounds('omega', -1, 1);

solver.setBounds('omegad', -0.2, 0.2);

solver.setInitialBounds('p', [-10000;0;-10000], [10000;0;-100]);

% assign initial guess
ig = solver.initialGuess();
gridpoints = linspace(0, 1, N+1);

ig{1}.add('p', gridpoints, ref_p);
ig{1}.add('v', gridpoints, ref_v);

rotRefCell = num2cell(reshape(ref_R,3,3, size(ref_R,2)), [1,2]);
ig{1}.add('R', gridpoints, rotRefCell);

% solve
[sol,gridpoints] = solver.solve(ig);

%% plot solution
t_states = gridpoints.states.value;
t_controls = gridpoints.controls.value;
t_collocation = gridpoints.integrator(:).value;

traj_p = sol.states.p.value;
traj_v = sol.states.v.value;
traj_omega = sol.states.omega.value;
traj_R = sol.states.R.value;

traj_lambda = sol.controls.lambda.value;
traj_omegad  = sol.controls.omegad.value;

figure;hold on;grid on;
subplot(5,1,1);plot(t_states,traj_p');  ylabel('p');legend({'x','y','z'});
subplot(5,1,2);plot(t_states,traj_v'); ylabel('v');legend({'x','y','z'});
subplot(5,1,3);plot(t_states,traj_omega');  ylabel('w');legend({'x','y','z'});
subplot(5,1,4);plot(t_controls,traj_omegad');  ylabel('dw');legend({'x','y','z'});
subplot(5,1,5);plot(t_controls,traj_lambda');  ylabel('lambda');


airspeed = zeros(N+1,1);
groundSpeed = diag(sqrt(traj_v'*traj_v));

windNavAtAltitude = zeros(3,N+1);
alpha = zeros(N+1,1);
beta = zeros(N+1,1);
for k=1:N+1
  p = traj_p(:,k);
  windNavAtAltitude(:,k) = awe.model.wind_at_altitude(conf.wind,p);
  R = traj_R{k};
  v = traj_v(:,k);
  [airspeed(k),alpha(k),beta(k)] = awe.model.aerodynamic_angles( v, R, windNavAtAltitude(:,k) );
end

% aircraft data
figure;hold on;grid on;
subplot(4,1,1);plot(airspeed);  ylabel('airspeed');
subplot(4,1,2);plot(groundSpeed);  ylabel('groundSpeed');
subplot(4,1,3);plot(alpha*180/pi);  ylabel('angle of attack deg');
subplot(4,1,4);plot(beta*180/pi);  ylabel('side slip angle deg');

% constraints satisfaction
%
% cTraj = [];
% dcTraj = [];
% for k=1:N+1
%   state = sol.states(:,:,k);
%   ic = system.icFun.evaluate(state.value,0);
%   cTraj   = [cTraj,;ic];
% end
% 
% figure;hold on;grid on;
% plot(cTraj');

% Plot reference trajectory -----------------------------------------
figure;
awe.plot.trajectory(ref_p, ref_v, ref_R, ones(size(ref_p,2), 1));

% figure;hold on;grid on;
% plot3(traj_p(1,:),traj_p(2,:),traj_p(3,:));
% plot3(traj_p(1,1),traj_p(2,1),traj_p(3,1),'go');
% plot3(traj_p(1,end),traj_p(2,end),traj_p(3,end),'ro');
% plot3(ref_p(1,:),ref_p(2,:),ref_p(3,:),'k');
% axis equal

% Plot solution trajectory ---------------------------------------------------
traj_R = reshape(cell2mat(traj_R),9,[]);
awe.plot.trajectory(traj_p, traj_p, traj_R, ones(size(traj_p,2), 1));

% 3d animation
%
% ts = vars.get('time').value/ (CONTROL_INTERVALS+1);
% viewStruct = CreatePlotView(ts,ts,fig);
% UpdatePlotView( viewStruct, pTraj, rotationNav)
