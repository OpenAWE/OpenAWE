function point_constraints(ch, k, K, x, conf)

% initial condition
if k ==1
  
  tether_ic = awe.model.rigid_tether_ic(x.p,x.v,conf.tether_length);
  ch.add(tether_ic, '==', 0);
  
  cR = x.R.'*x.R - eye(3);
  cR = [cR(:,1);cR([2,3],2);cR(3,3)];
  ch.add(cR, '==', 0);
  
%   ch.add(x.p, '==', x.p0);
%   ch.add(x.v, '==', x.v0);
%   ch.add(x.R, '==', x.R0);
%   ch.add(x.omega, '==', x.omega0);
end

% periodicity
% if (k==K)
%   p0 = x.p0;
%   v0 = x.v0;
%   R0 = x.R0;
%   omega0 = x.omega0;
%   
%   pF = x.p;
%   vF = x.v;
%   RF = x.R;
%   omegaF = x.omega;
%   
%   [yaw0,pitch0,roll0] = awe.model.yaw_pitch_roll(R0);
%   [yawF,pitchF,rollF] = awe.model.yaw_pitch_roll(RF);
%   
%   ch.add(p0, '==', pF);
%   ch.add(v0, '==', vF);
%   
%   ch.add(yaw0,'==',yawF);
%   ch.add(pitch0,'==',pitchF);
%   ch.add(roll0,'==',rollF);
%   
%   ch.add(omega0, '==', omegaF);
% 
% end

