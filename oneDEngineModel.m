
function dprop = oneDEngineModel(x,prop)

%prop is a vector of :T , U, rho, p
% Solve this by space stepping along the combustor, using an rk4 ODE solver
% over each step
mdot = 2; % total mass flow





% Geometry, fairly arbitrary right now
Linlet = 1;
Lcomb = 1;
Lnoz = 1;

if x <= Linlet % inlet
    A = 0.65 - 0.1*x;
    dAdx = -0.1; %duct area gradient
elseif Lcomb + Linlet >= x && x > Linlet % combustor
    A = 0.55;
    dAdx = 0.; %duct area gradient
elseif x > Lcomb + Linlet % nozzle
    A = 0.55 + 0.1*x;
    dAdx = 0.1; %duct area gradient
end
    


hf = 14.31*prop(1); % fuel specific enthalpy of H2

R = 8.3144598 ; % universal gas constant kJ/kg-mol-k

MWO = 15.9994 ; %molecular weight O2
MWO2 = 2*MWO;
MWN = 14.0067; %molecular weight N2
MWN2 = 2*MWN;
MWH2O = 18.01528; %molecular weight H2O
MWH = 1.00794; %molecular weight H

fracO2i = 0.21; %mass fraction oxygen initially
fracN2i = 0.79; %mass fraction N2 initially (just assuming all the rest is N2)
mdotO2i = fracO2i*mdot; % mass flow rate of O2

mdotf = mdotO2i*2*MWH/MWO; % mass flow rate fuel over entire combustor, moles to make H20




%NASA polynomial coefficients O2
a1O2 = 3.78245636E+00;
a2O2 = -2.99673416E-03;
a3O2 = 9.84730201E-06  ;  
a4O2 = -9.68129509E-09;
a5O2 = 3.24372837E-12;
a6O2 = -1.06394356E+03;

hO2 = R*prop(1)/MWO2*(a1O2 + a2O2/2*prop(1) +a3O2/3*prop(1)^2 +a4O2/4*prop(1)^3 +a5O2/5*prop(1)^4 + a6O2/prop(1)); % enthalpy O2, given by NASA polynomial
cpO2 = R/MWO2*(a1O2 + a2O2*prop(1) +a3O2*prop(1)^2 +a4O2*prop(1)^3 +a5O2*prop(1)^4); % specific heat O2

%NASA polynomial coefficients N2
a1N2 = 0.03298677E+02;
a2N2 = 0.14082404E-02;
a3N2 = -0.03963222E-04  ; 
a4N2 = 0.05641515E-07;
a5N2 = -0.02444854E-10;
a6N2 = -0.10208999E+04;
 
hN2 = R*prop(1)/MWN2*(a1N2 + a2N2/2*prop(1) +a3N2/3*prop(1)^2 +a4N2/4*prop(1)^3 +a5N2/5*prop(1)^4 + a6N2/prop(1)); % enthalpy N2, given by NASA polynomial
cpN2 = R/MWN2*(a1N2 + a2N2*prop(1) +a3N2*prop(1)^2 +a4N2*prop(1)^3 +a5N2*prop(1)^4);

%NASA polynomial coefficients H2O
a1H2O = 4.19864056E+00;
a2H2O = -2.03643410E-03;
a3H2O = 6.52040211E-06  ;
a4H2O = -5.48797062E-09;
a5H2O = 1.77197817E-12;
a6H2O = -3.02937267E+04;
 
hH2O = R*prop(1)/MWH2O*(a1H2O + a2H2O/2*prop(1) +a3H2O/3*prop(1)^2 +a4H2O/4*prop(1)^3 +a5H2O/5*prop(1)^4 + a6H2O/prop(1)); % enthalpy H2O, given by NASA polynomial
cpH2O = R/MWH2O*(a1H2O + a2H2O*prop(1) +a3H2O*prop(1)^2 +a4H2O*prop(1)^3 +a5H2O*prop(1)^4);


% assuming that only mass change is fuel mass
if x <= Linlet % inlet
dmdotdx = 0;
fracO2 = fracO2i;
fracH2O = 0;
dMWdx = 0 ;

elseif Lcomb + Linlet >= x && x > Linlet % combustor
dmdotdx = mdotf/Lcomb; % change in mass flow rate, assumed to be even over combustor
fracO2 = fracO2i - fracO2i*(x - Linlet)/Lcomb; 
fracH2O =  fracO2i*(x - Linlet)/Lcomb; 
dMWdx = fracO2i/Lcomb*(MWH2O - MWO2)  ;

elseif x > Lcomb + Linlet % nozzle
fracO2 = 0; 
fracH2O =  fracO2i;
dmdotdx = 0;
dMWdx = 0 ;
end

MW = MWN2*fracN2i + MWO2*fracO2 + MWH2O*fracH2O; % molecular weight

 % molecular weight gradient

h = hN2*fracN2i + hO2*fracO2 + hH2O*fracH2O; % total enthalpy
h0 = h + prop(2)^2/2; % stagnation enthalpy



cp = cpN2*fracN2i + cpO2*fracO2 + cpH2O*fracH2O; % specific heat



Tw = 300; % wall temp, just assuming that this stays constant

Pr = 0.71; % Prandtl number

Delta = 4*A/(2*pi*(sqrt(A)/pi)); %hydraulic diameter, bottom term is perimeter of duct ( I am assuming circular the whole way through, this is bad)


% U = 1; %velocity

% dUdx = 1; %velocity gradient


dhO2 = R/MWO2*(a1O2 + a2O2*prop(1) +a3O2*prop(1)^2 +a4O2*prop(1)^3 +a5O2*prop(1)^4 );
dhN2 = R/MWN2*(a1N2 + a2N2*prop(1) +a3N2*prop(1)^2 +a4N2*prop(1)^3 +a5N2*prop(1)^4 );
dhH2O = R/MWH2O*(a1H2O + a2H2O*prop(1) +a3H2O*prop(1)^2 +a4H2O*prop(1)^3 +a5H2O*prop(1)^4 );

dhdT = dhN2*fracN2i + dhO2*fracO2 + dhH2O*fracH2O;


A0 = 0.65; % initial area

dhdp = (A0-1)*prop(1)/prop(4)*dhdT;% enthalpy - pressure gradient


% just set these to zero, not sure how to calculate them
dhdphi = 0; %enthalpy change with  equivalence ratio

dphidx = 0; %enthalpy gradient



gamma = 1.4; % specific heat ratio

M = 6; % Mach no

% T = 1; % temperature





% rho = 1; % density

mu = 1.983 * 10e-5 ; %dynamic viscosity, air at room temp

Re = prop(2)*Delta*prop(3)/mu; %Reynolds no. for a duct

cf = 0.0576*Re^(-1/5); %skin friction coefficient at point from leading edge

% cf = 1;

Taw = prop(1)*(1 + Pr^(1/3)*0.5*M^2*(gamma-1)); %adiabatic wall temp for turbulent




% cf = 1;




dprop = zeros(4,1);



 % dUdx
 alpha = 1/prop(2)*(1 - gamma*M^2*A0 + prop(2)^2/prop(1)*dhdT^-1);
 
 dprop(2) = 1/alpha*(-1/A*dAdx + 1/mdot*dmdotdx*(1 + gamma*M^2*A0 + 1/prop(1)*dhdT^-1 * (hf - h0)) - 1/MW*dMWdx + 2*cf/Delta*(gamma*M^2*A0 - cp*(Taw - Tw)/Pr^(2/3) /prop(1) *dhdT^-1) - 1/prop(1)*dhdphi*dphidx*dhdT^-1);

% drhodx
 dprop(3) = prop(3)*(1/mdot*dmdotdx - 1/prop(2)*dprop(2) - 1/A*dAdx);

 % dpdx
 dprop(4) = -gamma*prop(3)*M^2*(1/prop(2)*dprop(2) + 0.5*4*cf/Delta + 1/mdot*dmdotdx);

% dTdx
 dprop(1) = dhdT^-1*((hf/mdot*dmdotdx - 2*cf*cp*(Taw - Tw)/(Pr^(2/3)*Delta*A) - h0/mdot*dmdotdx - prop(2)*dprop(2)) - dhdp*dprop(2) - dhdphi*dphidx);
end

