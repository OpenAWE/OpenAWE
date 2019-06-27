import casadi.*

% Parameters =============================================================
p                          = struct;
p.massPlane                = 2;
p.AspectRatio              = 10;
p.airDensity               = 1.23; 
p.gravity                  = 9.81; 
p.wingArea                 = 0.5; 
p.massCart                 = 1;
p.aeroDragCoefficientPlane = 0.04;                                  % for StreamLine Body
p.aeroDragCoefficientCart  = 0.6;                                   % long cylinder      (static value)   Df = 0.82
p.rollDragCoefficientCart  = 0.06;                                  % Tire - wet Asphalt (dynamic value) Df = 0.6  
p.AreaCart                 = 0.1^2;
mPlane        = p.massPlane;
AR            = p.AspectRatio;
rho           = p.airDensity;
g             = p.gravity;
Sref          = p.wingArea;
mCart         = p.massCart;
cD_plane      = p.aeroDragCoefficientPlane;                
cDaeroCart    = p.aeroDragCoefficientCart;                
cDrollCart    = p.rollDragCoefficientCart; 
Acart         = p.AreaCart;



% numer of segments/nodes
N = 20;
L0 = 0.1;

cD_node = 1;
mNode   = 1/N;


% array to collect variables on the way
variables = {};

alpha = SX.sym('alpha');


% cart
pCart = SX.sym('p_cart',1);
vCart = SX.sym('v_cart',1);
variables = [variables, {pCart, vCart}];

% forces on the cart 
FrollDragCart  = -cDrollCart*mCart*g;                  % roll drag
FaeroDragCart  = -0.5*rho*vCart^2*cDaeroCart*Acart;   % air resistance
FCart          = FrollDragCart + FaeroDragCart;


FNodes = [];
pNodes = [];
vNodes = [];

% N cable mass nodes with acting drag and tension forces
for k=1:N
  pNode = SX.sym(['p_' num2str(k)],2);
  vNode = SX.sym(['v_' num2str(k)],2);
  variables = [variables, {pNode, vNode}];
  
  % forces on the cable mass nodes 
  V              = norm_2(vNode);
  Fdrag          = 0.5*V^2*cD_node;
  Fgravity       = [0;mNode*g];
  
  FNode          = 0.0013*Fdrag+Fgravity;
  
  FNodes = [FNodes,FNode];
  pNodes = [pNodes,pNode];
  vNodes = [vNodes,vNode];
  
end

% aircraft 
pPlane = SX.sym('p_plane',2);
vPlane = SX.sym('v_plane',2);
variables = [variables, {pPlane, vPlane}];

CL             = 2*pi*alpha*(10/12);          % Lift Coefficient
CD             = cD_plane+CL^2/(AR*pi);       % Drag Coefficient
V              = norm_2(vPlane);   % velocity

eL             = 1/V*[ vPlane(2);-vPlane(1)];     % Lift Direction
eD             = 1/V*[-vPlane(1);-vPlane(2)];     % Drag Coefficient

Flift          = 0.5*rho*V^2*CL*Sref*eL;      % Lift Force
Fdrag          = 0.5*rho*V^2*CD*Sref*eD;      % Drag Force
Fgravity       = [0;mPlane*g];                % Gravity Force
Faero          = Flift+Fdrag+Fgravity;        % Total Aircraft Force

% --------------------------------
% add tension forces

tensionVariables = {};

% tension force on cart and first node
tension = SX.sym('t_cart');
tensionVariables = [tensionVariables, {tension}];
FtCart    = (pNodes(:,1) - [pCart;0]) / norm_2(pNodes(:,1) - pCart) * tension; 

FCart = FCart + FtCart(1);
FNodes(:,1) = FNodes(:,1)-FtCart;

% tension forces on nodes
for k=1:N-1
  tension = SX.sym(['t_' num2str(k)]);
  tensionVariables = [tensionVariables, {tension}];
  FtNode = (pNodes(:,k+1) - pNodes(:,k)) / norm_2(pNodes(:,k+1) - pNodes(:,k)) * tension;
  FNodes(:,k) = FNodes(:,k) + FtNode;
  FNodes(:,k+1) = FNodes(:,k+1) - FtNode;
end

% tension between last node and plane
tension = SX.sym('t_plane', 1);
tensionVariables = [tensionVariables, {tension}];

FtPlane         = (pPlane - pNodes(:,end)) / norm_2(pPlane - pNodes(:,end)) * tension; 
Faero = Faero - FtPlane;
FNodes(:,end) = FNodes(:,end) + FtPlane;



% ode withoout constraint equations

aCart = FCart / mCart;
aNodes = FNodes ./ mNode;
aPlane = Faero /mPlane;




% constraint equations

% between cart and first node
pNode = pNodes(:,1);
vNode = vNodes(:,1);
aNode = aNodes(:,1);

C = norm_2([pCart;0]-pNode)^2 - L0^2;
dC = jacobian(C,[pCart;pNode]) * [vCart;vNode];
ddC = jacobian(dC,[pCart;pNode]) * [vCart;vNode] + jacobian(dC,[vCart;vNode]) * [aCart;aNode];

algebraic_equations = ddC;

% between nodes
for k=1:N-1
  pPrevNode = pNodes(:,k);
  vPrevNode = vNodes(:,k);
  aPrevNode = aNodes(:,k);

  pNode = pNodes(:,k+1);
  vNode = vNodes(:,k+1);
  aNode = aNodes(:,k+1);

  C = norm_2(pPrevNode-pNode)^2 - L0^2;
  dC = jacobian(C,[pPrevNode;pNode]) * [vPrevNode;vNode];
  ddC = jacobian(dC,[pPrevNode;pNode]) * [vPrevNode;vNode] + jacobian(dC,[vPrevNode;vNode]) * [aPrevNode;aNode];

  algebraic_equations = [algebraic_equations; ddC];
end

% between plane and last node
pLastNode = pNodes(:,end);
vLastNode = vNodes(:,end);
aLastNode = aNodes(:,end);

C = norm_2(pLastNode-pPlane)^2 - L0^2;
dC = jacobian(C,[pLastNode;pPlane]) * [vLastNode;vPlane];
ddC = jacobian(dC,[pLastNode;pPlane]) * [vLastNode;vPlane] + jacobian(dC,[vLastNode;vPlane]) * [aLastNode;aPlane];

algebraic_equations = [algebraic_equations; ddC];


% build up equation
equation = [vCart;aCart];
for k=1:N
  equation = [equation; vNodes(:,k); aNodes(:,k)];
end
equation = [ equation; vPlane; aPlane];



% initialize integrator
           
dae = struct;
dae.x = vertcat(variables{:});
dae.z = vertcat(tensionVariables{:});
dae.p = alpha;
dae.ode = equation;
dae.alg = algebraic_equations;
           
integratorOptions = struct;
integratorOptions.tf = 0.02;
% integratorOptions.linear_solver = 'csparse';
% integratorOptions.linear_solver_type = 'user_defined';
integratorOptions.steps_per_checkpoint = 1;
integratorOptions.abstol = 1e-8;
modelIntegrator = integrator('integrator','idas',dae,integratorOptions);


% setup initial state

initialPlaneHeight = -50;

initialState =[];

pCart0 = 0;
vCart0 = 15;

initialState = [pCart0; vCart0];

for k=1:N
  pNodek0 = [0; k * initialPlaneHeight / (N+1)];
  vNodek0 = [15; 0];
  
  initialState = [initialState; pNodek0; vNodek0];
end

pPlane0 = [0;-50];
vPlane0 = [15;0];

initialState = [initialState; pPlane0; vPlane0];


initialAlgebraicVariables = zeros(N+1,1);

integratorParams = struct;
integratorParams.x0 = initialState;
integratorParams.p = 5 * pi/180;
integratorParams.z0 = initialAlgebraicVariables;

tsim = [];
Xsim = [];
tic
for i=1:1000
  
  integrationStep = modelIntegrator(integratorParams);
  
  integratorParams.x0 = integrationStep.xf;
  integratorParams.z0 = integrationStep.zf;
  
  tsim = [tsim;i*integratorOptions.tf];
  Xsim = [Xsim;full(integrationStep.xf)'];
  
end
toc
VisualizeMultiNode(tsim,Xsim);





