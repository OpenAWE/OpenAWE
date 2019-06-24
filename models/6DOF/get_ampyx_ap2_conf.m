function parametersAmpyxAP2 = get_ampyx_ap2_conf()

parametersAmpyxAP2 = struct;

% setup parameters
parametersAmpyxAP2.cableDiameter    = 0.0025;
parametersAmpyxAP2.cableDensity     = 0.0046;
parametersAmpyxAP2.wingArea         = 3;
parametersAmpyxAP2.wingSpan         = 5.5;
parametersAmpyxAP2.chord            = 0.55;
parametersAmpyxAP2.mass             = 36.8;
parametersAmpyxAP2.inertia          = [25,0,-0.47;0,32,0;-0.47,0,56];

% winch propterties
parametersAmpyxAP2.winchRadius      = 0.15;
parametersAmpyxAP2.winchInertia     = 8.5;

% cable parameters
parametersAmpyxAP2.attachmentPointPositionInBody = [0;0;0.05];
parametersAmpyxAP2.springStiffness               = 1e11;
parametersAmpyxAP2.springDamping                 = 3000;
parametersAmpyxAP2.dragCoefficient               = 1.2;

% setup aerodynamical coefficients
parametersAmpyxAP2.coefficients = struct;
parametersAmpyxAP2.coefficients.CX_alpha  = [2.5549; 0.4784; -0.0293];
parametersAmpyxAP2.coefficients.CZ_alpha  = [5.7736; -5.0676; -0.5526];
parametersAmpyxAP2.coefficients.dCX.dq    = [4.4124;-0.6029];
parametersAmpyxAP2.coefficients.dCY.db    = [0.0936; -0.0299; -0.1855];
parametersAmpyxAP2.coefficients.dCY.dp    = [0.0496; -0.0140; -0.1022];
parametersAmpyxAP2.coefficients.dCY.dr    = [0.1368; 0.1695];
parametersAmpyxAP2.coefficients.dCZ.dq    = [6.1486; 0.1251; -7.5561];

% setup wind and environment
parametersAmpyxAP2.wind = struct;
parametersAmpyxAP2.wind.atBaseAltitude = [8;0;0];
parametersAmpyxAP2.wind.baseAltitude   = 6.5;
parametersAmpyxAP2.wind.exponent       = 0.12;

parametersAmpyxAP2.airDensity        = 1.225;
parametersAmpyxAP2.gravNav           = [0;0;9.81];

% cable number of segments
parametersAmpyxAP2.nSegments         = 10;
