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



  end % methods
end
