classdef PowerOptimizationOCP < OclOCP
  properties
    system
    N
    T
    
    pRef
    vRef
    rotRef
  end
  methods
    function self = PowerOptimizationOCP(system,N,T)
      self.N = N;
      self.T = T;
      self.system = system;
      
      [self.pRef,self.vRef,~,self.rotRef] = getReferenceFlightPath(N,T);
      
    end
    function cost = discreteCost(self,nlpVars)
     
      
      pTraj = nlpVars.get('states').get('positionNav');
      cost = 0;
      for k=1:self.N+1
        error = pTraj(:,k)-self.pRef(:,k);
        cost = cost + error'*error;
      end
      
      mu = nlpVars.parameters.mu;
      cost = nlpVars.parameters.w_referenceTracking * (1-mu)*cost/nlpVars.time;
      
%       cost = cost + 1e-5*pTraj(2,1);
      
    end
    function pathCosts(self,state,algState,controls,time,endTime,params)
      bodyAngularAccel = controls.get('bodyAngularAccel');
      ddl = controls.get('ddl');
      self.addPathCost(params.w_bodyAngularAccel * (bodyAngularAccel'*bodyAngularAccel)/endTime);
      self.addPathCost(params.w_ddl * ddl^2/endTime);
      
      
      positionNav = state.get('positionNav');
      velocityNav = state.get('velocityNav');
      rotBodyToNav = state.get('rotBodyToNav');
      
      windNavAtAltitude = GetWindAtAltitude(self.system.modelParams.wind,positionNav);
      
      [~,~,beta] = AerodynamicAngles( velocityNav, rotBodyToNav, windNavAtAltitude);
      self.addPathCost(params.w_beta * beta^2/endTime);
      
      self.addPathCost(-1e-4*state.dl/endTime)
    end
    function arrivalCosts(self,state,endTime,params)
      mu = params.get('mu');
      integratedWork = state.get('integratedWork');
      self.addArrivalCost( - params.w_integratedWork * mu*integratedWork/endTime);
    end
    function pathConstraints(self,state,algState,controls,time,params)    
      
      positionNav = state.get('positionNav');
      velocityNav = state.get('velocityNav');
      rotBodyToNav = state.get('rotBodyToNav');
      
      windNavAtAltitude = GetWindAtAltitude(self.system.modelParams.wind,positionNav);
      
      
      [airspeed,alpha,beta] = AerodynamicAngles( velocityNav, rotBodyToNav, windNavAtAltitude);
      self.addPathConstraint(airspeed,'<=',params.MAX_AIRSPEED);
      self.addPathConstraint(airspeed,'>=',params.MIN_AIRSPEED);
      self.addPathConstraint(alpha,'<=',params.MAX_ALPHA);
      self.addPathConstraint(alpha,'>=',params.MIN_ALPHA);
      self.addPathConstraint(beta,'<=',params.MAX_BETA);
      self.addPathConstraint(beta,'>=',params.MIN_BETA);
      
      % line angle <= 30 deg  (or) cos(line angle) >= 60
      downVec = rotBodyToNav(:,3);
      winchVec = -positionNav/sqrt(positionNav(1)^2+positionNav(2)^2+positionNav(3)^2);
      self.addPathConstraint(dot(downVec,winchVec),'>=',cos(60*pi/180));
      
      % tension <= 5000 N
      lambda = algState.get('lambda');
      l = state.get('l');
      self.addPathConstraint(lambda*l,'<=',params.MAX_TENSION);
      self.addPathConstraint(lambda*l,'>=',params.MIN_TENSION);
      
    end    
    function boundaryConditions(self,state0,stateF,params)     
      % consistency conditions
      ic = self.system.getInitialConditions(state0.value,params.value);
      self.addBoundaryCondition(ic,'==',0);
      
      % 
      positionNav = state0.get('positionNav');
      velocityNav = state0.get('velocityNav');
      rotBodyToNav = state0.get('rotBodyToNav');
      bodyAngularRate = state0.get('bodyAngularRate');    
      
      
      positionEnd = stateF.get('positionNav');
      self.addBoundaryCondition(positionNav,'==',positionEnd);
      
      velocityEnd = stateF.get('velocityNav');
      self.addBoundaryCondition(velocityNav,'==',velocityEnd);
      
      bodyAngularRateEnd = stateF.get('bodyAngularRate');
      self.addBoundaryCondition(bodyAngularRate,'==',bodyAngularRateEnd);
      
      % periodic toration
      rotBodyToNavEnd = stateF.get('rotBodyToNav');
      [yawEnd,pitchEnd,rollEnd] = GetYawPitchRoll(rotBodyToNavEnd);
      [yawStart,pitchStart,rollStart] = GetYawPitchRoll(rotBodyToNav);
      
      self.addBoundaryCondition(yawStart,'==',yawEnd);
      self.addBoundaryCondition(pitchStart,'==',pitchEnd);
      self.addBoundaryCondition(rollStart,'==',rollEnd);
      
    end
  end
end

