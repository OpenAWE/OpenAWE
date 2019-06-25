function initial_condition(ch, x)
  l  = x.l;
  ld = x.ld;

  p  = x.p;
  v  = x.v;
  R  = x.R;

  tether_ic = awe.models.full.rigid_tether_ic(p,v,l,ld);

  cR = R.'*R - eye(3);
  cR = [cR(:,1);cR([2,3],2);cR(3,3)];

  ch.add(tether_ic);
  ch.add(cR);
end