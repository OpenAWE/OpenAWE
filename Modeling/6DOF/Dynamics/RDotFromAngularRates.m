function RDot = RDotFromAngularRates(rotation, bodyAngularRate)

omegaX    = bodyAngularRate(1);
omegaY    = bodyAngularRate(2);
omegaZ    = bodyAngularRate(3);


RDot = rotation * [0,       omegaZ,  -omegaY; ...
                   -omegaZ, 0,       omegaX ; ...
                   omegaY,  -omegaX, 0      ].';