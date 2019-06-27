function dynamics(eh, x, z, u, conf)
  p     = x.p;
  v     = x.v;
  R     = x.R;
  omega = x.omega;
  
  omegad = u.omegad;
  lambda = u.lambda;

  RDot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';
            
  a = awe.model.linear_acceleration(p, v, R, omega, lambda, conf);

  eh.setODE('p',      v);
  eh.setODE('v',      a);
  eh.setODE('R',      RDot);
  eh.setODE('omega',  omegad);

%   tether_eq = awe.model.rigid_tether_equation(p, v, accel);
%   eh.setAlgEquation(tether_eq);
