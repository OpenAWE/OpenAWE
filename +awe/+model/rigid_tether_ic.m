function ic  = rigid_tether_ic(p,v,l)
  % tether initial conditions
  % fixed tether length
  % constant velocity
  ic = (p.'*p) - l^2;;
end
