function xd = dynamics(x, conf, controller)
  p     = x(1:3);
  v     = x(4:6);
  R     = reshape(x(7:15), 3, 3);
  omega = x(16:18);
  
  [omegad, lambda] = controller(x,conf);

  R_dot = R * [0,         omega(3),  -omega(2); ...
              -omega(3), 0,         omega(1) ; ...
              omega(2),  -omega(1), 0        ].';
            
  a = awe.model.linear_acceleration(p, v, R, omega, lambda, conf);
  
  xd = [v;a;reshape(R_dot,9,1);omegad];
