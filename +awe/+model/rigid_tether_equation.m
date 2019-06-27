function r = rigid_tether_equation(p,v,a)
  % constant velocity
r = dot(p,a)+dot(v,v);
