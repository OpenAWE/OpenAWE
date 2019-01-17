classdef PowerOptimizationSystem < OclSystem
  
  properties
    modelParams
    view
  end
  
  methods   
        
    function self = PowerOptimizationSystem()
      self.modelParams = GetModelParameters;
    end
    
    function setupVariables(self)
      self.addState('positionNav',[3,1]);
      self.addState('velocityNav',[3,1]);
      self.addState('rotBodyToNav',[3,3]);
      self.addState('bodyAngularRate',[3,1]);
      
      self.addControl('bodyAngularAccel',[3,1]);
      
      self.addState('integratedWork',[1 1]);
      
      self.addState('l',[1,1]);
      self.addState('dl',[1,1]);
      self.addControl('ddl',[1,1]);
      self.addAlgVar('lambda',[1,1]);     
      
      self.addParameter('wingArea',[1 1]);
      self.addParameter('wingSpan',[1 1]);
      self.addParameter('chord'   ,[1 1]);
      self.addParameter('mass'    ,[1 1]);
      
      % optimal control parameters
      self.addParameter('mu',[1 1]);
      self.addParameter('w_referenceTracking',[1 1]);
      self.addParameter('w_tensionTracking',[1 1]);
      self.addParameter('w_bodyAngularAccel',[1 1]);
      self.addParameter('w_ddl',[1 1]);
      self.addParameter('w_beta',[1 1]);
      self.addParameter('w_integratedWork',[1 1]);
      
      self.addParameter('MAX_AIRSPEED',[1 1]);
      self.addParameter('MIN_AIRSPEED',[1 1]);
      self.addParameter('MAX_ALPHA',[1 1]);
      self.addParameter('MIN_ALPHA',[1 1]);
      self.addParameter('MAX_BETA',[1 1]);
      self.addParameter('MIN_BETA',[1 1]);
      self.addParameter('MAX_TENSION',[1 1]);
      self.addParameter('MIN_TENSION',[1 1]);
      
    end
    function setupEquation(self,states,algVars,controls,parameters)
      % Get access to the system parameters 
      wingArea = parameters.get('wingArea');
      wingSpan = parameters.get('wingSpan');
      chord    = parameters.get('chord');
      mass     = parameters.get('mass');
       
      positionNav     = states.get('positionNav');
      velocityNav     = states.get('velocityNav');
      rotBodyToNav    = states.get('rotBodyToNav');
      bodyAngularRate = states.get('bodyAngularRate');
      
      bodyAngularAccel = controls.get('bodyAngularAccel');
      
      l               = states.get('l');
      dl              = states.get('dl');
      ddl             = controls.get('ddl');
      lambda          = algVars.get('lambda');
      
      windNav = GetWindAtAltitude(self.modelParams.wind,positionNav);
      [airspeed,alpha,beta] = AerodynamicAngles(velocityNav,rotBodyToNav,windNav);

      aeroForcesBody     = AerodynamicForces( alpha, ...
                                              beta, ...
                                              airspeed, ...
                                              bodyAngularRate, ....
                                              self.modelParams.coefficients, ...
                                              self.modelParams.airDensity, ...
                                              wingSpan, ...
                                              wingArea, ...
                                              chord);
                                            
      tetherForce = RigidTetherForce( positionNav,alpha,...
                                      rotBodyToNav,l,lambda, ...
                                      airspeed,self.modelParams.airDensity, ...
                                      self.modelParams.cableDiameter, ...
                                      self.modelParams.dragCoefficient, ...
                                      self.modelParams.cableDensity); 
       
      forces      = rotBodyToNav * aeroForcesBody + tetherForce + self.modelParams.gravNav;
      
      accelNav    = LinearPointMassDynamics(forces,mass);
                                         
      RDot        = RDotFromAngularRates(rotBodyToNav, bodyAngularRate);
      
      tetherEquation = RigidTetherEquation(positionNav,velocityNav,accelNav,l,dl,ddl);

      
      self.setODE('positionNav',      velocityNav);
      self.setODE('velocityNav',      accelNav);
      self.setODE('rotBodyToNav',     RDot);
      self.setODE('bodyAngularRate',  bodyAngularAccel);
      
      self.setODE('l',                dl);
      self.setODE('dl',               ddl);
      
      self.setODE('integratedWork',   l*dl*lambda);
      
      self.setAlgEquation(tetherEquation);
      
    end
    
    function initialCondition(self,state,parameters)
      l  = state.get('l').value;
      dl = state.get('dl').value;
      
      p  = state.get('positionNav').value;   
      v  = state.get('velocityNav').value; 
      R  = state.get('rotBodyToNav').value;
      
      tetherInitialCondition = RigidTetherInitialCondition(p,v,l,dl);
      
      cR = R'*R - eye(3);
      cR = [cR(:,1);cR([2,3],2);cR(3,3)];
      
      self.setInitialCondition(tetherInitialCondition);
      self.setInitialCondition(cR);
    end
    
  end % methods
end

