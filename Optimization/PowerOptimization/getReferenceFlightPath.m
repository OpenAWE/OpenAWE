function [pTraj,vTraj,aTraj,RTraj] = getReferenceFlightPath(N,T,verbose)
%% compute initial guess for Homotopy algoritm
% computeReference(N,T)
% computeReference(N,T,verbose)

if nargin < 3
  verbose = false;
end

% Define pattern parameter ------------------------------------------------
lineRadiusGuess   = 400; % [m]                     
circleRadiusGuess = 200; % [m]  
minAltitude       = 100;  % [m]

r      = circleRadiusGuess;             % radious of lemniscate
h      = sqrt(lineRadiusGuess^2 - r^2); % distante beetween winch and center of lemniscate
nTurns = 1;
direction = -1;

pTraj = zeros(3,N);
vTraj = zeros(3,N);
aTraj = zeros(3,N);
RTraj = zeros(9,N);

for k=1:N+1
  parametric = (k-1)/N;

  % path following lemniscate
  theta = nTurns*2*pi*parametric;
  theta = direction*(theta - pi);

  thetaDot = nTurns*direction*2*pi/ T; % T is the time for the loop [s]

  % lemniscate parametric function
  xyzCircleFrame       = [h ;  r*sin(theta)           ;    -0.5*r*sin(2*theta)          ];
  xyzDotCircleFrame    = [0 ;  r*cos(theta)*thetaDot  ;    -r*cos(2*theta)*thetaDot     ];
  xyzDotDotCircleFrame = [0 ; -r*sin(theta)*thetaDot^2;    2*r*sin(2*theta)*thetaDot^2  ];                


  phi = asin(r/lineRadiusGuess); % [rad] rotate so it's above ground
  phi = phi + asin(minAltitude/lineRadiusGuess);

  R_c2n = [ cos(phi), 0, sin(phi);
                   0, 1,        0;
           -sin(phi), 0, cos(phi)]; 

  % pre-allocation
  xyz       = R_c2n*xyzCircleFrame;
  xyzDot    = R_c2n*xyzDotCircleFrame;
  xyzDotDot = R_c2n*xyzDotDotCircleFrame;

  ex = xyzDot/norm(xyzDot);       % x-axis point nose aircraft
  ez = -xyz/norm(xyz);      % z-axis point down aircraft
  ey = cross(ez,ex);              % y-axis point towards wing aircraft
  R = [ex,ey,ez];

  pTraj(:,k) = xyz;
  vTraj(:,k) = xyzDot;
  aTraj(:,k) = xyzDotDot;
  RTraj(:,k) = reshape(R,9,1);

end
