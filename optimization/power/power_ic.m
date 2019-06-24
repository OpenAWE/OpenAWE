function power_ic(ch, x, ~)
  l  = x.get('l').value;
  dl = x.get('dl').value;

  p  = x.get('positionNav').value;
  v  = x.get('velocityNav').value;
  R  = x.get('rotBodyToNav').value;

  tetherInitialCondition = RigidTetherInitialCondition(p,v,l,dl);

  cR = R'*R - eye(3);
  cR = [cR(:,1);cR([2,3],2);cR(3,3)];

  ch.add(tetherInitialCondition);
  ch.add(cR);
end