function initialCondition  = RigidTetherInitialCondition(p,v,l,dl)
initialCondition = [p'*p - l^2;p'*v - l*dl];
end

