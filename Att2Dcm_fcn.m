function DCM = Att2Dcm(ATT)
%#codegen

phi=ATT(1);
theta=ATT(2);
psi=ATT(3);

DCM = [ cos(theta)*cos(psi)                             cos(theta)*sin(psi)                            -sin(theta)          ; 
        sin(theta)*cos(psi)*sin(phi)-sin(psi)*cos(phi)  sin(theta)*sin(psi)*sin(phi)+cos(psi)*cos(phi)  cos(theta)*sin(phi) ;
        sin(theta)*cos(psi)*cos(phi)+sin(psi)*sin(phi)  sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi)  cos(theta)*cos(phi)   ];
   
   