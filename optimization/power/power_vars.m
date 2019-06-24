function power_vars(vh)
vh.addState('positionNav',[3,1]);
vh.addState('velocityNav',[3,1]);
vh.addState('rotBodyToNav',[3,3]);
vh.addState('bodyAngularRate',[3,1]);

vh.addControl('bodyAngularAccel',[3,1]);

vh.addState('integratedWork',[1 1]);

vh.addState('l',[1,1]);
vh.addState('dl',[1,1]);
vh.addControl('ddl',[1,1]);
vh.addAlgVar('lambda',[1,1]);

vh.addParameter('wingArea',[1 1]);
vh.addParameter('wingSpan',[1 1]);
vh.addParameter('chord'   ,[1 1]);
vh.addParameter('mass'    ,[1 1]);

% optimal control parameters
vh.addParameter('mu',[1 1]);
vh.addParameter('w_referenceTracking',[1 1]);
vh.addParameter('w_tensionTracking',[1 1]);
vh.addParameter('w_bodyAngularAccel',[1 1]);
vh.addParameter('w_ddl',[1 1]);
vh.addParameter('w_beta',[1 1]);
vh.addParameter('w_integratedWork',[1 1]);

vh.addParameter('MAX_AIRSPEED',[1 1]);
vh.addParameter('MIN_AIRSPEED',[1 1]);
vh.addParameter('MAX_ALPHA',[1 1]);
vh.addParameter('MIN_ALPHA',[1 1]);
vh.addParameter('MAX_BETA',[1 1]);
vh.addParameter('MIN_BETA',[1 1]);
vh.addParameter('MAX_TENSION',[1 1]);
vh.addParameter('MIN_TENSION',[1 1]);
