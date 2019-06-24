classdef PowerOptimizationSystem < OclSystem

  properties
    modelParams
    view
  end

  methods (Static)

    function self = PowerOptimizationSystem()
      self.modelParams = GetModelParameters;
    end

    function setupVariables(self)


    end
    function setupEquation(self,states,algVars,controls,parameters)
      

    end

    function initialConditions(self,state,parameters)
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
