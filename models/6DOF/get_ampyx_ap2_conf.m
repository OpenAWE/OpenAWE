function r = get_ampyx_ap2_conf()

r = struct;

% setup parameters
r.cableDiameter    = 0.0025;
r.cableDensity     = 0.0046;
r.wingArea         = 3;
r.wingSpan         = 5.5;
r.chord            = 0.55;
r.mass             = 36.8;
r.inertia          = [25,0,-0.47;0,32,0;-0.47,0,56];

% winch propterties
r.winchRadius      = 0.15;
r.winchInertia     = 8.5;

% cable parameters
r.attachmentPointPositionInBody = [0;0;0.05];
r.springStiffness               = 1e11;
r.springDamping                 = 3000;
r.dragCoefficient               = 1.2;

% setup aerodynamical coefficients
r.coefficients = struct;
r.coefficients.CX_alpha  = [2.5549; 0.4784; -0.0293];
r.coefficients.CZ_alpha  = [5.7736; -5.0676; -0.5526];
r.coefficients.dCX.dq    = [4.4124;-0.6029];
r.coefficients.dCY.db    = [0.0936; -0.0299; -0.1855];
r.coefficients.dCY.dp    = [0.0496; -0.0140; -0.1022];
r.coefficients.dCY.dr    = [0.1368; 0.1695];
r.coefficients.dCZ.dq    = [6.1486; 0.1251; -7.5561];
