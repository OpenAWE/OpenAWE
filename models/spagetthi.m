function spagetthi

g = 9.81;
N = 20;
rpaMass = 333;  % rpa = remote piloted aircraft
cableDensityUpper = 0.0444;  

spagetthiLength = 25;
L0 = spagetthiLength/N;
segmentMass = L0*cableDensityUpper;

d = 0.00777;    
A = (d/2)^2*pi;
E = 1e11;
springCoeff = E*A/L0;             % N/m  

liftForce = 33333/rpaMass;

preTensionAccel = 3333/segmentMass;

activeNodes = 0;

rpaPosition0 = [0;L0];
rpaVelocity0 = [0;0];

positions0 = zeros(2,N);
velocities0  = zeros(2,N);  

x = pack(rpaPosition0,rpaVelocity0,positions0,velocities0);

[tout,xout] = ode45(@ode,(0:0.05:1.3),x);

figure;
for l=1:length(tout)-1
  tic;
  x = xout(l,:)';
  [rpaPosition,rpaVelocity,positions,velocities] = unpack(x);
  
  hold off
  plot([positions(1,:),rpaPosition(1)],[positions(2,:),rpaPosition(2)],'-')
  hold on
  plot(rpaPosition(1,:),rpaPosition(2,:),'rx-')
  
  
  xlim([-30,10])
  ylim([-10,30])
  pause(mean(diff(tout))-toc);
  title(tout(l))
  drawnow
end

keyboard

  function dx = ode(~,x)
    [rpaPosition,rpaVelocity,positions,velocities] = unpack(x);
    [rpaAcceleration,accelerations] = dynamics(rpaPosition,rpaVelocity,positions,velocities);
    dx = pack(rpaVelocity,rpaAcceleration,velocities,accelerations);
  end

  function x = pack(rpaPosition,rpaVelocity,positions,velocities)
    x = zeros(5+4*N-1,1);
    x(1:2) = rpaPosition;
    x(3:4) = rpaVelocity;
    x(5:5+2*N-1) = reshape(positions,2*N,1);
    x(5+2*N:5+4*N-1) = reshape(velocities,2*N,1); 
  end

  function [rpaPosition,rpaVelocity,positions,velocities] = unpack(x)
    rpaPosition = x(1:2);
    rpaVelocity = x(3:4);
    positions = reshape(x(5:5+2*N-1),2,N);
    velocities = reshape(x(5+2*N:5+4*N-1),2,N);
  end

  function [rpaAcceleration,accelerations] = dynamics(rpaPosition,rpaVelocity,positions,velocities)

    accelerations  = zeros(2,N);
    for k=1:N-1

      pk = positions(:,k);
      pknext = positions(:,k+1);
      
      distance = norm(pknext-pk);

      displacement = distance-L0;     % m
      if displacement>0
        tensionForce = springCoeff * displacement;
        ak = tensionForce * (pknext-pk) / distance / segmentMass;
        aknext = -tensionForce * (pknext-pk) / distance / segmentMass;
        
        activeNodes = max(activeNodes,N-k+1);
      else
        ak = 0;
        aknext = 0;
      end
      
      % tension and gravity on all active nodes
      if k > N-activeNodes 
        accelerations(:,k)   = accelerations(:,k) + ak;
        accelerations(:,k+1) = accelerations(:,k+1) + aknext - [0;g];
      end
      
      % pretension on single active node
      if (k == N-activeNodes+1) && (accelerations(1,k) < -preTensionAccel)
        accelerations(1,k)  = accelerations(1,k) + preTensionAccel;
      end

    end

    pN = positions(:,N);
    distance = norm(rpaPosition-pN);
    
    displacement = distance-L0;     % m
    if displacement>1e-3
      tensionForce = springCoeff * displacement;
      tensionvec = tensionForce * (rpaPosition-pN) / distance;
      aN = tensionvec / segmentMass;
      rpaAcceleration = [-tensionvec(1)/rpaMass;0];
      
      if activeNodes == 0 && (aN(1) < -preTensionAccel)
        aN(1)  = aN(1) + preTensionAccel;
      end
      
    else
      aN = 0;
      rpaAcceleration = [0;0];
    end
    
    accelerations(:,N) = accelerations(:,N)+aN;    
    accelerations(:,1) = [0;0];

    if norm(rpaVelocity) < 25
      rpaAcceleration = rpaAcceleration + [-3*g;0];
    end
    
    if rpaPosition(1) < -18
      rpaAcceleration(2) = rpaAcceleration(2) + liftForce;
    end
    
  end

end