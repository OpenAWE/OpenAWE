function [pTraj,vTraj,RTraj] = reference_path(N, T)

% Define pattern parameter ------------------------------------------------
lineRadiusGuess   = 400; % [m]
circleRadiusGuess = 200; % [m]
minAltitude       = 100; % [m]

r      = circleRadiusGuess;             % radius of lemniscate
h      = sqrt(lineRadiusGuess^2 - r^2); % distance beetween winch and center of lemniscate
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

  % lemniscate parametric function on circular path
  p_circle = [h ;  r*sin(theta)           ;    -0.5*r*sin(2*theta)          ];
  v_circle = [0 ;  r*cos(theta)*thetaDot  ;    -r*cos(2*theta)*thetaDot     ];
  a_circle = [0 ; -r*sin(theta)*thetaDot^2;    2*r*sin(2*theta)*thetaDot^2  ];

  % rotate circle by phi so that it is above the ground
  phi = asin(r/lineRadiusGuess); % [rad] 
  phi = phi + asin(minAltitude/lineRadiusGuess);

  R_c2n = [ cos(phi), 0, sin(phi);
                   0, 1,        0;
           -sin(phi), 0, cos(phi)];

  p = R_c2n*p_circle;
  v = R_c2n*v_circle;
  a = R_c2n*a_circle;

  ex = v/norm(v);       % x-axis point nose aircraft
  ez = -p/norm(p);      % z-axis point down aircraft
  ey = cross(ez,ex);    % y-axis point towards wing aircraft
  R = [ex,ey,ez];

  pTraj(:,k) = p;
  vTraj(:,k) = v;
  aTraj(:,k) = a;
  RTraj(:,k) = reshape(R,9,1);

end
