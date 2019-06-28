function [K,S,E] = lqr_gains(conf)

  p = casadi.SX.sym('p',3,1);
  v = casadi.SX.sym('v',3,1);
  Rv = casadi.SX.sym('R',9,1);
  omega = casadi.SX.sym('omega',3,1);
  
  omegad = casadi.SX.sym('omegad',3,1);
  lambda = casadi.SX.sym('lambda',1,1);

  x = vertcat(p,v,Rv,omega);
  u = vertcat(omegad, lambda);
  
  R = reshape(Rv, 3, 3);
  
  R_dot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';
            
  a = awe.model.linear_acceleration(p, v, R, omega, lambda, conf);
  xd = [v;a;reshape(R_dot,9,1);omegad];
  
  jx = casadi.Function('jx', {x,u}, {jacobian(xd,x)});
  ju = casadi.Function('ju', {x,u}, {jacobian(xd,u)});
  
  % solve for xd = 0
  f = p(2)^2 + 1e-3*norm(reshape(R-eye(3), 9, 1)) + (p(1)+300)^2 + (p(3)+100)^2+norm(xd);
  g = reshape(R'*R-eye(3), 9, 1);
  
  nlp = struct('x', [x;u], 'f', f, 'g', g);
  S = casadi.nlpsol('S', 'ipopt', nlp);
  r = S('x0',rand(22,1), 'lbg', 0, 'ubg', 0);
  
  y_trim = full(r.x);
  
  A = full(jx(y_trim(1:18),y_trim(19:22)));
  B = full(ju(y_trim(1:18),y_trim(19:22)));
  
  
  % add state for p3 reference (-100)
  Abar = blkdiag(A,1);
  Bbar = vertcat(B, zeros(1,4));
  
  Q = zeros(19);
  Q(3,3) = 1;
  Q(3,19) = -1;
  Q(19,3) = -1;
  Q(19,19) = 1;
  
  R = 1e-6*eye(4,4);
  
  sys = ss(Abar,Bbar,eye(19), 0)
  minreal(sys)
  
  
  [K,S,E] = lqr(Abar, Bbar, Q, R);