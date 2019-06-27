%% Plane and Cart with cable shape in DAE formulation [2D]
% Version: Matlab 2014a/ casadi v3.0.0. rc2
% Author:  Marie Curie PhD student koenemann & Giovanni Licitra
% Data:    16-02-2016

import casadi.*
% Parameters =============================================================
p                          = struct;

p.massPlane                = 2;
p.massCart                 = 1;
p.massCable                = 1;

p.AspectRatio              = 10;
p.AreaCart                 = 0.1^2;

p.airDensity               = 1.23; 
p.gravity                  = 9.81; 
p.wingArea                 = 0.5; 
p.aeroDragCoefficientPlane = 0.04;                                  % for StreamLine Body
p.aeroDragCoefficientCart  = 10*0.6;                                   % long cylinder      (static value)   Df = 0.82
p.rollDragCoefficientCart  = 0.06;                                  % Tire - wet Asphalt (dynamic value) Df = 0.6  


mPlane        = p.massPlane;
mCart         = p.massCart;
mCable        = p.massCable;

AR            = p.AspectRatio;
Acart         = p.AreaCart;

rho           = p.airDensity;
g             = p.gravity;
Sref          = p.wingArea;
cDplane       = p.aeroDragCoefficientPlane;                
cDaeroCart    = p.aeroDragCoefficientCart;                
cDrollCart    = p.rollDragCoefficientCart;



% numer of segments/nodes
N = 1;            % number of nodes
lengthCable   = 50;
lengthSegment = lengthCable/N;          % length of segment

cableDiam     = 0.005;
cD_node = 0.82;       % tether drag coefficient 
mNode   = 1/N;     % mass node 


simulationTime = 0;  % seconds
simulationTimestep = 0.02;





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
  Fdrag          = -0.5*V^2*cD_node*cableDiam*lengthSegment * vNode / V;
  Fgravity       = [0;mNode*g];
  
  FNode          = Fdrag+Fgravity;
  
  FNodes = [FNodes,FNode];
  pNodes = [pNodes,pNode];
  vNodes = [vNodes,vNode];
  
end

% aircraft is last node
pPlane        = pNodes(:,end);
vPlane        = vNodes(:,end);

CL             = 2*pi*alpha*(10/12);          % Lift Coefficient
CD             = cDplane+CL^2/(AR*pi);         % Drag Coefficient
V              = norm_2(vPlane);   % velocity

eL             = 1/V*[ vPlane(2);-vPlane(1)];     % Lift Direction
eD             = 1/V*[-vPlane(1);-vPlane(2)];     % Drag Coefficient

Flift          = 0.5*rho*V^2*CL*Sref*eL;      % Lift Force
Fdrag          = 0.5*rho*V^2*CD*Sref*eD;      % Drag Force
Fgravity       = [0;mPlane*g];                % Gravity Force
Faero          = Flift+Fdrag+Fgravity;        % Total Aircraft Force

FNodes(:,end) = FNodes(:,end) + Faero;
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



% ode withoout constraint equations

aCart = FCart / mCart;
aNodes = FNodes ./ mNode;
aNodes(:,end) = FNodes(:,end) ./ mPlane;

% constraint equations

% between cart and first node
pNode = pNodes(:,1);
vNode = vNodes(:,1);
aNode = aNodes(:,1);



C   = norm_2([pCart;0]-pNode)^2 - lengthSegment^2;
dC  = jtimes(C,[pCart;pNode], [vCart;vNode]);
ddC = jtimes(dC,[pCart;pNode], [vCart;vNode]) + jtimes(dC,[vCart;vNode], [aCart;aNode]);

algebraic_equations = ddC;
constraints = C;
dConstraints = dC;

% between nodes
for k=1:N-1
  pPrevNode = pNodes(:,k);
  vPrevNode = vNodes(:,k);
  aPrevNode = aNodes(:,k);

  pNode = pNodes(:,k+1);
  vNode = vNodes(:,k+1);
  aNode = aNodes(:,k+1);

  C = norm_2(pPrevNode-pNode)^2 - lengthSegment^2;
  dC = jtimes(C,[pPrevNode;pNode], [vPrevNode;vNode]);
  ddC = jtimes(dC,[pPrevNode;pNode], [vPrevNode;vNode]) + jtimes(dC,[vPrevNode;vNode], [aPrevNode;aNode]);

  algebraic_equations = [algebraic_equations; ddC];
  constraints = [constraints; C];
  dConstraints = [dConstraints; dC];
end


% build up equation
equation = [vCart;aCart];
for k=1:N
  equation = [equation; vNodes(:,k); aNodes(:,k)];
end


% initialize integrator
           
dae = struct;
dae.x = vertcat(variables{:});
dae.z = vertcat(tensionVariables{:});
dae.p = alpha;
dae.ode = equation;
dae.alg = algebraic_equations;

opts = struct;
opts.jit = false;

% n_nodes(dae.ode)
% n_nodes(dae.alg)

% opts.jit_options.flags = char('-O3','');
dae_fun = Function('dae',dae,char('x','z','p','t'),char('ode','alg','quad'),opts);
% J = fun.jacobian();

integratorOptions = struct;
integratorOptions.tf = simulationTimestep;
%integratorOptions.linear_solver = 'csparse';
%integratorOptions.linear_solver_type = 'user_defined';
integratorOptions.steps_per_checkpoint = 1;
integratorOptions.abstol = 1e-8;
% integratorOptions.exact_jacobian = true;
% integratorOptions.calc_ic = true;
%integratorOptions.linear_solver_type = 'iterative';
% integratorOptions.implicit_solver = 'newton';
%integratorOptions.number_of_finite_elements = 1;
%integratorOptions.number_of_finite_elements = 1;
% integratorOptions.jit = true;
% integratorOptions.compiler = 'gcc';
integratorOptions.max_num_steps = 30000;
modelIntegrator = integrator('integrator','idas',dae_fun,integratorOptions);




%% SIMULATION

% setup initial state
initialPlaneHeight = -lengthCable;

initialState =[];

pCart0 = 0;
vCart0 = 15;

initialState = [pCart0; vCart0];

for k=1:N
  pNodek0 = [0; k * initialPlaneHeight / N];
  vNodek0 = [15; 0];
  
  initialState = [initialState; pNodek0; vNodek0];
end

initialAlgebraicVariables = zeros(N,1);

integratorParams = struct;
integratorParams.x0 = initialState;
integratorParams.p = 4 * pi/180;
integratorParams.z0 = initialAlgebraicVariables;

tsim = [];
Xsim = [];

tic
for i=1:simulationTime/simulationTimestep
  
  integrationStep = modelIntegrator(integratorParams);
  
  integratorParams.x0 = integrationStep.xf;
  integratorParams.z0 = integrationStep.zf;
  
  tsim = [tsim;i*integratorOptions.tf];
  Xsim = [Xsim;full(integrationStep.xf)'];
  
end
toc
% VisualizeMultiNode(tsim,Xsim);





%% COLLOCATION METHOD
nx = numel(dae.x);
nz = numel(dae.z);
nu = numel(dae.p);


T = 10.0;
horizonLength = 20; % Caution; mumps linear solver fails when too large

% Degree of interpolating polynomial
collocationDegree = 4;

% Get collocation points
collocationRoots = collocationPoints(collocationDegree, 'radau');

collfun = simpleColl(dae,collocationRoots,T/horizonLength);
collfun = collfun.expand();


% variables along horizon
stateTrajectory = {};
for i=1:horizonLength+1
   stateTrajectory{i} = MX.sym(['X_' num2str(i)],nx);
end
collocationStates = {};
algebraicStateTrajectory = {};
controlTrajectory = {};
for i=1:horizonLength
   collocationStates{i}          = MX.sym(['XC_' num2str(i)],nx,collocationDegree);
   algebraicStateTrajectory{i}   = MX.sym(['Z_' num2str(i)],nz,collocationDegree);
   controlTrajectory{i}          = MX.sym(['U_' num2str(i)],nu);
end

variablesBlock = struct();
variablesBlock.X  = Sparsity.dense(nx,1);
variablesBlock.XC = Sparsity.dense(nx,collocationDegree);
variablesBlock.Z  = Sparsity.dense(nz,collocationDegree);
variablesBlock.U  = Sparsity.dense(nu,1);

invariants = Function('invariants',{dae.x},{[constraints;dConstraints]});


% Simple bounds on states
lbx = {};
ubx = {};

% List of constraints
g = {};

% List of all decision variables (determines ordering)
V = {};
for k=1:horizonLength
  % Add decision variables
  V = {V{:} casadi_vec(variablesBlock,'X',stateTrajectory{k},'XC',collocationStates{k},'Z',algebraicStateTrajectory{k},'U',controlTrajectory{k})};
  
  if k==1
    % Bounds at t=0
    thisStateLB = initialState;
    thisStateUB = initialState;
    
    lbx = {lbx{:} casadi_vec(variablesBlock,-inf,'X',thisStateLB)};
    ubx = {ubx{:} casadi_vec(variablesBlock,inf, 'X',thisStateUB)};
  else
    % Bounds on state
    % add height>=0 here
    lbx = {lbx{:} casadi_vec(variablesBlock,-inf)};
    ubx = {ubx{:} casadi_vec(variablesBlock,inf)};
  end
  % Obtain collocation expressions
  coll_out = collfun({stateTrajectory{k},collocationStates{k},algebraicStateTrajectory{k},controlTrajectory{k}});

  g = {g{:} coll_out{2}};         % collocation constraints
  g = {g{:} stateTrajectory{k+1}-coll_out{1}}; % gap closing

end
  
V = {V{:} stateTrajectory{end}};

% Bounds for final t
finalStateLB = -inf*ones(numel(initialState),1);
finalStateUB = inf*ones(numel(initialState),1);
lbx = {lbx{:} finalStateLB};
ubx = {ubx{:} finalStateUB};


% cost function


controlVector = vertcat(controlTrajectory{:});

inv_out = invariants({stateTrajectory{1}});
nlp = struct('x',vertcat(V{:}), 'f',(controlVector'*controlVector), 'g', [inv_out{1};vertcat(g{:})]);

nlpfun = Function('nlp',nlp,char('x','p'),char('f','g'));

solver = nlpsol('solver','ipopt',nlp);


x0_guess = initialState;
z_guess  = zeros(nz,1);
u_guess  = 0;
x0 = [repmat([repmat(x0_guess,collocationDegree+1,1);repmat(z_guess,collocationDegree,1);u_guess],horizonLength,1);x0_guess];

args = struct;
args.x0 = x0;
args.lbx = vertcat(lbx{:});
args.ubx = vertcat(ubx{:});
args.lbg = 0;
args.ubg = 0;

res = solver(args);

results  = full(res.x);



