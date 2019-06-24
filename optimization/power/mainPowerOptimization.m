CONTROL_INTERVALS = 40;    % horizon discretization
T = 44;

system = PowerOptimizationOCP.system;
wind = system.modelParams.wind;
wind.atBaseAltitude = [8;0;0];
ocl = OclSolver(system,PowerOptimizationOCP);

[pRef,vRef,aRef,rotRef] = getReferenceFlightPath(PowerOptimizationOCP.N,PowerOptimizationOCP.T);

ocl.setParameter('wingArea',PowerOptimizationOCP.system.modelParams.wingArea);
ocl.setParameter('wingSpan',PowerOptimizationOCP.system.modelParams.wingSpan);
ocl.setParameter('chord'   ,PowerOptimizationOCP.system.modelParams.chord);
ocl.setParameter('mass'    ,PowerOptimizationOCP.system.modelParams.mass);

ocl.setParameter('w_referenceTracking'  ,1     );
ocl.setParameter('w_bodyAngularAccel'   ,1e2   );
ocl.setParameter('w_ddl'                ,1e-2  );
ocl.setParameter('w_beta'               ,1e2   );
ocl.setParameter('w_integratedWork'     ,1e-3  );

ocl.setParameter('MAX_AIRSPEED'         ,32    );
ocl.setParameter('MIN_AIRSPEED'         ,13    );
ocl.setParameter('MAX_ALPHA'            ,0.16   );
ocl.setParameter('MIN_ALPHA'            ,-0.1   );
ocl.setParameter('MAX_BETA'             ,0.3   );
ocl.setParameter('MIN_BETA'             ,-0.3   );
ocl.setParameter('MAX_TENSION'          ,5000  );
ocl.setParameter('MIN_TENSION'          ,10    );

ocl.setParameter('time',T);

ocl.setBounds('l', 1, 700);
ocl.setBounds('dl',-15,20);

ocl.setBounds('positionNav',[-10000;-10000;-10000],[10000;10000;-100]);
ocl.setBounds('velocityNav',-60,60);
ocl.setBounds('rotBodyToNav',-1.1,1.1);
ocl.setBounds('bodyAngularRate',-1,1);

ocl.setBounds('bodyAngularAccel',-0.2,0.2);
ocl.setBounds('ddl',-2.3,2.4);

ocl.setInitialBounds('integratedWork',0,0);
ocl.setInitialBounds('positionNav',[-10000;0;-10000],[10000;0;-100]);


% assign initial guess
vars = ocl.getInitialGuess();

vars.get('states').get('positionNav').set(pRef);
vars.get('time').set(T);
vars.get('states').get('l').set(400);
vars.get('states').get('velocityNav').set(vRef);

rotRefCell = squeeze(num2cell(reshape(rotRef,3,3,size(rotRef,2)),[1,2]));
vars.get('states').get('rotBodyToNav').set(rotRefCell);

%vars.get('integrator').get('algVars').get('lambda').set(1);

% solve
ocl.setParameter('mu', 0);
[vars,times] = ocl.solve(vars);

keyboard
%
%ocl.setParameter('mu',1);
%[vars,times] = ocl.solve(vars);
%
%ocl.setParameter('time',5, T+20);
%vars = ocl.solve(vars);

% plot solution
times   = linspace(0,vars.get('time').value,CONTROL_INTERVALS+1);
pTraj   = vars.get('states').get('positionNav').value;
vTraj   = vars.get('states').get('velocityNav').value;
wTraj   = vars.get('states').get('bodyAngularRate').value;
rotTraj = vars.get('states').get('rotBodyToNav').value;
%zTraj   = vars.get('integrator').get('algVars',options.nlp.collocationOrder).get('lambda').value;
lTraj   = vars.get('states').get('l').value;
dlTraj  = vars.get('states').get('dl').value;

ddlTraj = vars.get('controls').get('ddl').value;
dwTraj  = vars.get('controls').get('bodyAngularAccel').value;

figure;hold on;grid on;
plot3(pTraj(1,:),pTraj(2,:),pTraj(3,:));
plot3(pTraj(1,1),pTraj(2,1),pTraj(3,1),'go');
plot3(pTraj(1,end),pTraj(2,end),pTraj(3,end),'ro');
plot3(pRef(1,:),pRef(2,:),pRef(3,:),'k');
axis equal

% average power
%mechanicalWork = zTraj.*lTraj(2:end).*dlTraj(2:end);
%integratedWork = cumtrapz(times(2:end),mechanicalWork)/times(end);
%integratedWorkState = vars.get('states').get('integratedWork').value;
%integratedWorkState = integratedWorkState/times(end);
%figure;hold on;grid on;
%plot(times(2:end),mechanicalWork,'b');
%plot(times(2:end),integratedWork,'r');
%plot(times,integratedWorkState,'g');
%legend('mechanical work','average power (trapezoidal)','avarage power (integrator)')



figure;hold on;grid on;
subplot(8,1,1);plot(pTraj');  ylabel('p');legend({'x','y','z'});
subplot(8,1,2);plot(vTraj'); ylabel('v');legend({'x','y','z'});
subplot(8,1,3);plot(wTraj');  ylabel('w');legend({'x','y','z'});
subplot(8,1,4);plot(dwTraj');  ylabel('dw');legend({'x','y','z'});
subplot(8,1,5);plot(lTraj');  ylabel('l');
subplot(8,1,6);plot(dlTraj');  ylabel('dl');
subplot(8,1,7);plot(ddlTraj');  ylabel('ddl');
%subplot(8,1,8);plot(zTraj');  ylabel('lambda');


airspeed = zeros(CONTROL_INTERVALS+1,1);
groundSpeed = diag(sqrt(vTraj'*vTraj));
%tension = zTraj .* diag(sqrt(pTraj(:,2:end)'*pTraj(:,2:end)))';

windNavAtAltitude = zeros(3,CONTROL_INTERVALS+1);
alpha = zeros(CONTROL_INTERVALS+1,1);
beta = zeros(CONTROL_INTERVALS+1,1);
for k=1:CONTROL_INTERVALS+1
  p = pTraj(:,k);
  windNavAtAltitude(:,k) = GetWindAtAltitude(system.modelParams.wind,p);
  R = rotTraj{k};
  v = vTraj(:,k);
  [airspeed(k),alpha(k),beta(k)] = AerodynamicAngles( v, R, windNavAtAltitude(:,k) );
end



figure;hold on;grid on;
subplot(5,1,1);plot(airspeed);  ylabel('airspeed');
subplot(5,1,2);plot(groundSpeed);  ylabel('groundSpeed');
%subplot(5,1,3);plot(tension');  ylabel('tension');
subplot(5,1,4);plot(alpha*180/pi);  ylabel('angle of attack deg');
subplot(5,1,5);plot(beta*180/pi);  ylabel('side slip angle deg');



cTraj = [];
dcTraj = [];
for k=1:CONTROL_INTERVALS+1
  state = vars.states(:,:,k);
  ic = system.icFun.evaluate(state.value,0);
  cTraj   = [cTraj,;ic];
end

figure;hold on;grid on;
plot(cTraj');


% Plot trajectory ---------------------------------------------------
fig = figure; hold on;grid on;

s = 10;

rotationNavToView = [1,0,0;
                     0,-1,0;
                     0,0,-1];

for k=1:CONTROL_INTERVALS

  R = vars.states(:,:,k).get('rotBodyToNav').value;
  pTrajView = rotationNavToView*pTraj(:,k);
  pRefView = rotationNavToView*pRef(:,k);
  vView = rotationNavToView * vars.states(:,:,k).get('velocityNav').value;
  rotView = rotationNavToView * R * rotationNavToView';
  rotRefView = rotationNavToView * reshape(rotRef(:,k),3,3) * rotationNavToView';

  % Plot cable --------------------------------------------------------
  if dlTraj(k) >= 0
    line([0,pTrajView(1)],[0,pTrajView(2)],[0,pTrajView(3)],'LineWidth',0.8,'Color','g','LineStyle',':');
  else
    line([0,pTrajView(1)],[0,pTrajView(2)],[0,pTrajView(3)],'LineWidth',0.8,'Color','r','LineStyle',':');
  end

  % Plot orientation --------------------------------------------------

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

end
hold off
% ts = vars.get('time').value/ (CONTROL_INTERVALS+1);
% viewStruct = CreatePlotView(ts,ts,fig);
% UpdatePlotView( viewStruct, pTraj, rotationNav)
