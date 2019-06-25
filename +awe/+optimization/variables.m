function variables(vh)
vh.addState('p',[3,1]);
vh.addState('v',[3,1]);
vh.addState('R',[3,3]);
vh.addState('omega',[3,1]);

vh.addControl('omegad',[3,1]);

vh.addState('iwork',[1 1]);

vh.addState('l',[1,1]);
vh.addState('ld',[1,1]);
vh.addControl('ldd',[1,1]);
vh.addAlgVar('lambda',[1,1]);

vh.addState('p0',[3,1]);
vh.addState('v0',[3,1]);
vh.addState('R0',[3,3]);
vh.addState('omega0',[3,1]);

vh.addState('time');

vh.addParameter('mu',[1 1]);

