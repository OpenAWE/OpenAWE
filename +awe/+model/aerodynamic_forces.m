function f = forces( alpha, ...
                                beta, ...
                                airspeed, ...
                                omega, ...
                                coefficients, ...
                                airDensity, ...
                                wingSpan, ...
                                wingArea, ...
                                chord)

qbar                        = airspeed^2 * 0.5 * airDensity;

CZ_alpha                    = polyval(coefficients.CZ_alpha,alpha);
CX_alpha                    = polyval(coefficients.CX_alpha,alpha);

CX_q                        = polyval(coefficients.dCX.dq,alpha);

CY_beta                     = polyval(coefficients.dCY.db,alpha);
CY_p                        = polyval(coefficients.dCY.dp,alpha);
CY_r                        = polyval(coefficients.dCY.dr,alpha);

CZ_q                        = polyval(coefficients.dCZ.dq,alpha);

phat                        = 0.5 * wingSpan / airspeed * omega(1);
qhat                        = 0.5 * chord / airspeed * omega(2);
rhat                        = 0.5 * wingSpan / airspeed * omega(3);

CX                          = CX_q * qhat;
CY                          = CY_beta * beta + CY_p * phat + CY_r * rhat;
CZ                          = CZ_q * qhat;
CX                          = CX + CX_alpha;
CZ                          = CZ + CZ_alpha;


FX_BODY                     = CX * qbar * wingArea;
FY_BODY                     = CY * qbar * wingArea;
FZ_BODY                     = CZ * qbar * wingArea;

f                           = [FX_BODY; FY_BODY; FZ_BODY];

end
