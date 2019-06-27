N = 60;
T = 44;

conf = awe.model.ampyx_ap2_conf();

% setup wind and environment
conf.wind = struct;
conf.wind.atBaseAltitude = [8;0;0];
conf.wind.baseAltitude   = 6.5;
conf.wind.exponent       = 0.12;

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
                    @(h,k,K,x,p) awe.optimization.point_constraints(h,k,K,x), ...
                    'N', N);

solver.setBounds('time', 0, T);

solver.setBounds('lambda', 0, 10000);

solver.setBounds('l', 1, 700);
solver.setBounds('ld', -15, 20);

solver.setBounds('p', [-10000; -10000; -10000], [10000; 10000; -100]);
solver.setBounds('v', -60, 60);
solver.setBounds('R', -1.1, 1.1);
solver.setBounds('omega', -1, 1);

solver.setBounds('omegad', -0.2, 0.2);
solver.setBounds('ldd', -2.3, 2.4);

solver.setInitialBounds('iwork', 0);
solver.setInitialBounds('time', 0);
solver.setInitialBounds('p', [-10000;0;-10000], [10000;0;-100]);

% assign initial guess
ig = solver.initialGuess();
gridpoints = linspace(0, 1, N+1);

ig{1}.add('time', [0 1], [0 T]);

ig{1}.add('p', gridpoints, ref_p);
ig{1}.add('v', gridpoints, ref_v);

rotRefCell = num2cell(reshape(ref_R,3,3, size(ref_R,2)), [1,2]);
ig{1}.add('R', gridpoints, rotRefCell);

ig{1}.add('l', gridpoints, 400);

ig{1}.add('p0', [0 1], ref_p(:,1));
ig{1}.add('v0', [0 1], ref_v(:,1));
ig{1}.add('R0', [0 1], reshape(ref_R(:,1),3,3))

g = ocl.simultaneous.getInitialGuessWithUserData(solver.stageList{1}, ig{1});

% solve
solver.setParameter('mu', 0);
[sol,gridpoints] = solver.solve(ig);

%
%ocl.setParameter('mu',1);
%[vars,times] = ocl.solve(vars);
%
%ocl.setParameter('time',5, T+20);
%vars = ocl.solve(vars);

% plot solution
traj_p = sol.states.p.value;
traj_v = sol.states.v.value;
traj_omega = sol.states.omega.value;
traj_R = sol.states.R.value;

traj_l   = sol.states.l.value;
traj_ld  = sol.states.ld.value;

traj_ldd = sol.controls.ldd.value;
traj_omegad  = sol.controls.omegad.value;

figure;hold on;grid on;
plot3(traj_p(1,:),traj_p(2,:),traj_p(3,:));
plot3(traj_p(1,1),traj_p(2,1),traj_p(3,1),'go');
plot3(traj_p(1,end),traj_p(2,end),traj_p(3,end),'ro');
plot3(ref_p(1,:),ref_p(2,:),ref_p(3,:),'k');
axis equal

% average power
%mechanicalWork = zTraj.*lTraj(2:end).*dlTraj(2:end);
%integratedWork = cumtrapz(times(2:end),mechanicalWork)/times(end);
%integratedWorkState = vars.states.get('integratedWork').value;
%integratedWorkState = integratedWorkState/times(end);
%figure;hold on;grid on;
%plot(times(2:end),mechanicalWork,'b');
%plot(times(2:end),integratedWork,'r');
%plot(times,integratedWorkState,'g');
%legend('mechanical work','average power (trapezoidal)','avarage power (integrator)')



figure;hold on;grid on;
subplot(8,1,1);plot(traj_p');  ylabel('p');legend({'x','y','z'});
subplot(8,1,2);plot(traj_v'); ylabel('v');legend({'x','y','z'});
subplot(8,1,3);plot(traj_omega');  ylabel('w');legend({'x','y','z'});
subplot(8,1,4);plot(traj_omegad');  ylabel('dw');legend({'x','y','z'});
subplot(8,1,5);plot(traj_l');  ylabel('l');
subplot(8,1,6);plot(traj_ld');  ylabel('dl');
subplot(8,1,7);plot(traj_ldd');  ylabel('ddl');
%subplot(8,1,8);plot(zTraj');  ylabel('lambda');


airspeed = zeros(N+1,1);
groundSpeed = diag(sqrt(traj_v'*traj_v));
%tension = zTraj .* diag(sqrt(pTraj(:,2:end)'*pTraj(:,2:end)))';

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



figure;hold on;grid on;
subplot(5,1,1);plot(airspeed);  ylabel('airspeed');
subplot(5,1,2);plot(groundSpeed);  ylabel('groundSpeed');
%subplot(5,1,3);plot(tension');  ylabel('tension');
subplot(5,1,4);plot(alpha*180/pi);  ylabel('angle of attack deg');
subplot(5,1,5);plot(beta*180/pi);  ylabel('side slip angle deg');



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


% Plot trajectory ---------------------------------------------------
fig = figure; hold on;grid on;

s = 10;

rotationNavToView = [1,0,0;
                     0,-1,0;
                     0,0,-1];

for k=1:N

  R = sol.states(:,:,k).R.value;
  pTrajView = rotationNavToView*traj_p(:,k);
  pRefView = rotationNavToView*ref_p(:,k);
  vView = rotationNavToView * sol.states(:,:,k).v.value;
  rotView = rotationNavToView * R * rotationNavToView';
  rotRefView = rotationNavToView * reshape(ref_R(:,k),3,3) * rotationNavToView';

  % Plot orientation of solution--------------------------------------------
  ex = s*rotView*[1;0;0];
  line([pTrajView(1),pTrajView(1)+ex(1)],...
   [pTrajView(2),pTrajView(2)+ex(2)],...
   [pTrajView(3),pTrajView(3)+ex(3)],'LineWidth',2,'Color','r','LineStyle','-');

  ey = s*rotView*[0;1;0];
  line([pTrajView(1),pTrajView(1)+ey(1)],...
   [pTrajView(2),pTrajView(2)+ey(2)],...
   [pTrajView(3),pTrajView(3)+ey(3)],'LineWidth',2,'Color','g','LineStyle','-');

  ez = s*rotView*[0;0;1];
  line([pTrajView(1),pTrajView(1)+ez(1)],...
   [pTrajView(2),pTrajView(2)+ez(2)],...
   [pTrajView(3),pTrajView(3)+ez(3)],'LineWidth',2,'Color','b','LineStyle','-');

  % Plot reference orientation --------------------------------------------------
  ex = s*rotRefView*[1;0;0];
  line([pRefView(1),pRefView(1)+ex(1)],...
   [pRefView(2),pRefView(2)+ex(2)],...
   [pRefView(3),pRefView(3)+ex(3)],'LineWidth',2,'Color','r','LineStyle','--');

  ey = s*rotRefView*[0;1;0];
  line([pRefView(1),pRefView(1)+ey(1)],...
   [pRefView(2),pRefView(2)+ey(2)],...
   [pRefView(3),pRefView(3)+ey(3)],'LineWidth',2,'Color','g','LineStyle','--');

  ez = s*rotRefView*[0;0;1];
  line([pRefView(1),pRefView(1)+ez(1)],...
   [pRefView(2),pRefView(2)+ez(2)],...
   [pRefView(3),pRefView(3)+ez(3)],'LineWidth',2,'Color','b','LineStyle','--');

  % plot flight direction
  v = vView;
  ev = s*v/norm(v);
  line([pTrajView(1),pTrajView(1)+ev(1)],...
   [pTrajView(2),pTrajView(2)+ev(2)],...
   [pTrajView(3),pTrajView(3)+ev(3)],'LineWidth',5,'Color','k','LineStyle','-');
 
  % Plot cable --------------------------------------------------------
  if traj_ld(k) >= 0
    line([0,pTrajView(1)],[0,pTrajView(2)],[0,pTrajView(3)],'LineWidth',0.8,'Color','g','LineStyle',':');
  else
    line([0,pTrajView(1)],[0,pTrajView(2)],[0,pTrajView(3)],'LineWidth',0.8,'Color','r','LineStyle',':');
  end
end
legend({'forward', 'wing', 'down'})
hold off
% ts = vars.get('time').value/ (CONTROL_INTERVALS+1);
% viewStruct = CreatePlotView(ts,ts,fig);
% UpdatePlotView( viewStruct, pTraj, rotationNav)
