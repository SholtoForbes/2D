
dhdT =1 

hf = 1; % fuel specific enthalpy

h0 = 1; % dont know what this is

mdot = 1; % total mass flow

dmdotfdx = 0 ; %change in fuel mass flow rate

cf = 1; %skin friction coefficient

cp = 1; % specific heat

Taw = 1; %adiabatic wall temp

Tw = 1; % wall temp

Pr = 0.71; % Prandtl number

Delta = 1; %hydraulic diameter

A = 1; %duct area

U = 1; %velocity

dUdx = 1; %velocity gradient

dhdp = 1;% enthalpy - pressure gradient

dpdx = 1; %pressure gradient

dhdphi = 1; %enthalpy change with  equivalence ratio

dphidx = 1; %enthalpy gradient

dTdx = dhdT^-1*((hf/mdot*dmdotfdx - 2*cf*cp*(Taw - Tw)/(Pr^(2/3)*Delta*A) - h0/mdot*dmdotfdx - U*dUdx) - dhdp*dpdx - dhdphi*dphidx)