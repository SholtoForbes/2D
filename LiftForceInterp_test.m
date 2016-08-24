function [AoA_interp, Drag_interp,Flap_pitchingmoment_interp] = LiftForceInterp(communicator,communicator_trim,const, Atmosphere,ThrustF_interp,FuelF_interp)
%Lift Force interpolator

%this module takes values from communicator and communicator-trim and finds
%appropriate AoA and flap deflection values to equalise a given 
%lift force. this allows for interps to be created that only
%require M and lift force to give flap deflection, AoA and total drag

%read communicator and create interpolatior / extrapolator interps
Cl_interp = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,3)); %find cl given M, AoA
Cd_interp = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,4)); % find Cd given M, AoA 
pitchingmoment_interp = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,11)); % find pitching moment given M, AoA changed from main code
flapdeflection_interp = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,3));
flapdrag_interp = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,5));
flaplift_interp = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,6));


[Mgrid_com2,alphagrid_com2] =  meshgrid(unique(sort(communicator(:,1))),unique(sort(communicator(:,2))));
CLgrid_com = Cl_interp(Mgrid_com2,alphagrid_com2);
CDgrid_com = Cd_interp(Mgrid_com2,alphagrid_com2);
Pitchgrid_com = pitchingmoment_interp(Mgrid_com2,alphagrid_com2);

A = 62.77; % reference area (m^2)

% golden sections method, to search for a variety of M and lift forces / q
%equalises the pitching moment of flap and body to calculate lift. works
%towards the correct AoA (and corresponding flap pitching moment)
liftarray = [];
for v = 1500:50:3000 % Velocity (m/s)
    for alt = [20000:1000:25000 25500:500:37000 38000:1000:50000] % Altitude (m)
        for Lift = [0:10000:70000 72000:2000:90000] % Lift force (N)   max mass of vehicle is 8755.1

            liftarray(end+1,1) = v;
            liftarray(end,2) = alt;
            liftarray(end,3) = Lift;
            c = spline( Atmosphere(:,1),  Atmosphere(:,5), alt); % Calculate speed of sound using atmospheric data
            rho = spline( Atmosphere(:,1),  Atmosphere(:,4), alt); % Calculate density using atmospheric data
            q = 0.5 * rho .* (v .^2); % Calculating Dynamic Pressure
            M = v./c; % Calculating Mach No (Descaled)
            
            %% Calculate Thrust Component ==================================

            if const == 1
            Efficiency = zeros(1,length(q));
            
            for i = 1:length(q)
                if q(i) < 50000
%                 if q(i) < 55000
%                 if q(i) < 45000
                Efficiency(i) = rho/(50000*2/v^2); % dont change this
                else
%                     Efficiency(i) = .9; % for 45kPa
                Efficiency(i) = 1; % for 50kPa
%                 Efficiency(i) = 1.1; % for 55kPa
            %     Efficiency(i) = 1.2; 
                end
            end
            else
                Efficiency = rho./(50000*2./v.^2); % linear rho efficiency, scaled to rho at 50000kpa
            end

            %% Determine AoA ==============================================================
            
    %         Alpha = Alpha_interp(M, Liftq/A); % first approximation of alpha using only body lift

            Alpha1 = 0;
            
            %Fuel Cost ===========================================================================

            Fueldt = FuelF_interp(M,Alpha1).*Efficiency;

            Isp = ThrustF_interp(M,Alpha1)./FuelF_interp(M,Alpha1); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================
            if Alpha1 > 2 && Alpha1 < 6
            Cl1 = interp2(Mgrid_com2,alphagrid_com2,CLgrid_com,M,Alpha1,'spline');
            body_pitchingmoment1 = interp2(Mgrid_com2,alphagrid_com2,Pitchgrid_com,M,Alpha1,'spline');% first approximation of pitchingmoment using only body lift
            else
            Cl1 = Cl_interp(M,Alpha1);
            body_pitchingmoment1 = pitchingmoment_interp(M, Alpha1);
            end
            
            Flap_lift1 = flaplift_interp(M,Alpha1,-body_pitchingmoment1);% first approximation of flap lift

            total_lift1 = Cl1*A*q + Flap_lift1 + Thrust*sin(deg2rad(Alpha1)); %first total lift force, with normalised dynamic pressure, this needs to iterate to equal the original liftq

            Alpha2 = 15; %first guesses of AoA
            %Fuel Cost ===========================================================================

            Fueldt = FuelF_interp(M,Alpha2).*Efficiency;

            Isp = ThrustF_interp(M,Alpha2)./FuelF_interp(M,Alpha2); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================
            if Alpha2 > 2 && Alpha2 < 6
            Cl2 = interp2(Mgrid_com2,alphagrid_com2,CLgrid_com,M,Alpha2,'spline');
            body_pitchingmoment2 = interp2(Mgrid_com2,alphagrid_com2,Pitchgrid_com,M,Alpha2,'spline');% first approximation of pitchingmoment using only body lift
            else
            Cl2 = Cl_interp(M,Alpha2);
            body_pitchingmoment2 = pitchingmoment_interp(M, Alpha2);
            end
            
            Flap_lift2 = flaplift_interp(M,Alpha2,-body_pitchingmoment2);% first approximation of flap lift

            total_lift2 = Cl2*A*q + Flap_lift2 + Thrust*sin(deg2rad(Alpha2));



            Alpha3 = Alpha2 - (-1+sqrt(5))/2*(Alpha2-Alpha1); %first golden section point

            Alpha4 = Alpha1 + (-1+sqrt(5))/2*(Alpha2-Alpha1); %first guesses of AoA


            while abs(Alpha4 - Alpha3) > 0.00005

            %Fuel Cost ===========================================================================

            Fueldt = FuelF_interp(M,Alpha3).*Efficiency;

            Isp = ThrustF_interp(M,Alpha3)./FuelF_interp(M,Alpha3); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================
            if Alpha3 > 2 && Alpha3 < 6
            Cl3 = interp2(Mgrid_com2,alphagrid_com2,CLgrid_com,M,Alpha3,'spline');
            body_pitchingmoment3 = interp2(Mgrid_com2,alphagrid_com2,Pitchgrid_com,M,Alpha3,'spline');% first approximation of pitchingmoment using only body lift
            else
            Cl3 = Cl_interp(M,Alpha3);
            body_pitchingmoment3 = pitchingmoment_interp(M, Alpha3);
            end
            
            Flap_lift3 = flaplift_interp(M,Alpha3,-body_pitchingmoment3);% first approximation of flap lift

            total_lift3 = Cl3*A*q + Flap_lift3 + Thrust*sin(deg2rad(Alpha3));


            %Fuel Cost ===========================================================================

            Fueldt = FuelF_interp(M,Alpha4).*Efficiency;

            Isp = ThrustF_interp(M,Alpha4)./FuelF_interp(M,Alpha4); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================
            if Alpha4 > 2 && Alpha4 < 6
            Cl4 = interp2(Mgrid_com2,alphagrid_com2,CLgrid_com,M,Alpha4,'spline');
            body_pitchingmoment4 = interp2(Mgrid_com2,alphagrid_com2,Pitchgrid_com,M,Alpha4,'spline');% first approximation of pitchingmoment using only body lift
            else
            Cl4 = Cl_interp(M,Alpha4);
            body_pitchingmoment4 = pitchingmoment_interp(M, Alpha4);
            end
            
            Flap_lift4 = flaplift_interp(M,Alpha4,-body_pitchingmoment4);% first approximation of flap lift

            total_lift4 = Cl4*A*q + Flap_lift4 + Thrust*sin(deg2rad(Alpha4));



                if abs(Lift - total_lift4) > abs(Lift - total_lift3)
                    Alpha2 = Alpha4;
                    Alpha4 = Alpha3;
                    Alpha3 = Alpha2 - (-1+sqrt(5))/2*(Alpha2-Alpha1);
                else
                    Alpha1 = Alpha3;
                    Alpha3 = Alpha4;
                    Alpha4 = Alpha1 + (-1+sqrt(5))/2*(Alpha2-Alpha1);
                end



            end

            flapdeflection = flapdeflection_interp(M,Alpha4,-body_pitchingmoment4);

            if Alpha4 > 2 && Alpha4 < 6
            Drag = interp2(Mgrid_com2,alphagrid_com2,CDgrid_com,M,Alpha4,'spline')*A*q +  flapdrag_interp(M,Alpha4,-body_pitchingmoment4);
%               Drag = Cd_interp(M,Alpha4)*A*q ; % changed to just body drag
            else
            Drag = Cd_interp(M,Alpha4)*A*q +  flapdrag_interp(M,Alpha4,-body_pitchingmoment4); 
            end
                
            liftarray(end,4) = Alpha4;

            liftarray(end,5) = flapdeflection;

            liftarray(end,6) = Drag;
            
            liftarray(end,7) = -body_pitchingmoment4;

        end
    end
end

% create interps
% given M and lift force / q , find AoA, flap deflection and total drag
% force / q
AoA_interp = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,4)); 
Drag_interp = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,6));
Flap_pitchingmoment_interp = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,7));

end