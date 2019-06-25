function r = rigid_tether_equation(p,v,a,l,ld,ldd)
  % constant velocity
r = dot(p,a)+dot(v,v)-l*ldd-ld^2;
