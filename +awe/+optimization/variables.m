function variables(vh)
vh.addState('p',[3,1]);
vh.addState('v',[3,1]);
vh.addState('R',[3,3]);
vh.addState('omega',[3,1]);

vh.addControl('omegad',[3,1]);

vh.addAlgVar('lambda',[1,1]);

