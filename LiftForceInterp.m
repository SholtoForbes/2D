function [AoA_spline, flapdeflection_spline, Drag_spline,Flap_pitchingmoment_spline] = LiftForceInterpOld(communicator,communicator_trim,const, Atmosphere,ThrustF_spline,FuelF_spline)
%Lift Force interpolator

%this module takes values from communicator and communicator-trim and finds
%appropriate AoA and flap deflection values to equalise a given q
%normalised lift force. this allows for splines to be created that only
%require M and lift force to give flap deflection, AoA and total drag


%read communicator and create interpolatior / extrapolator splines
Cl_spline = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,3)); %find cl given M, AoA
% flaplift_spline_2 = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,3),communicator_trim(:,6)); %find flap lift, given M, AoA, flap deflection

Cd_spline = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,4)); % find Cd given M, AoA 

% Alpha_spline = scatteredInterpolant(communicator(:,1),communicator(:,3),communicator(:,2)); % find AoA given M, Cl
 
pitchingmoment_spline = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,11)); % find pitching moment given M, AoA changed from main code
% pitchingmoment_spline = scatteredInterpolant(communicator(:,1),communicator(:,3),communicator(:,6));
flapdeflection_spline = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,3));

flapdrag_spline = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,5));

flaplift_spline = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,6));

A = 62.77; % reference area (m^2)

% golden sections method, to search for a variety of M and lift forces
%equalises the pitching moment of flap and body to calculate lift. works
%towards the correct AoA (and corresponding flap pitching moment)
liftarray = [];
for v = 1500:100:3000 % Velocity (m/s)
    for alt = 20000:1000:50000 % Altitude (m)
        for Lift = 0:5000:200000 % Lift force (N)   max mass of vehicle is 8755.1
            
            liftarray(end+1,1) = v;
            liftarray(end,2) = alt;
            liftarray(end,3) = Lift;
            
            c = spline( Atmosphere(:,1),  Atmosphere(:,5), alt); % Calculate speed of sound using atmospheric data

            rho = spline( Atmosphere(:,1),  Atmosphere(:,4), alt); % Calculate density using atmospheric data

            q = 0.5 * rho .* (v .^2); % Calculating Dynamic Pressure

            M = v./c; % Calculating Mach No (Descaled)
            
            %% Calculate Thrust Component ==================================

            if const == 1 || const == 14
                Efficiency = zeros(1,length(q));
                for i = 1:length(q)
                    if q(i) < 50000
                    Efficiency(i) = rho/(50000*2/v^2); % dont change this
                    else
                    Efficiency(i) = 1; % for 50kPa
                    end
                end
            elseif const == 12
                Efficiency = zeros(1,length(q));
                for i = 1:length(q)
                    if q(i) < 55000
                    Efficiency(i) = rho/(50000*2/v^2); % dont change this
                    else
                    Efficiency(i) = 1.1; % for 55kPa
                    end
                end
            elseif const == 13
                Efficiency = zeros(1,length(q));
                for i = 1:length(q)
                    if q(i) < 45000
                        Efficiency(i) = rho/(50000*2/v^2); % dont change this
                    else
                        Efficiency(i) = .9; % for 45kPa
                    end
                end
            elseif const == 3 || const == 31
            Efficiency = rho./(50000*2./v.^2); % linear rho efficiency, scaled to rho at 50000kpa
            end



            %% Determine AoA ==============================================================
            
            
            

    %         Alpha = Alpha_spline(M, Liftq/A); % first approximation of alpha using only body lift

            Alpha1 = 0;
            
            %Fuel Cost ===========================================================================

            Fueldt = FuelF_spline(M,Alpha1).*Efficiency;

            Isp = ThrustF_spline(M,Alpha1)./FuelF_spline(M,Alpha1); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================

            Cl1 = Cl_spline(M,Alpha1);

            body_pitchingmoment1 = pitchingmoment_spline(M, Alpha1);% first approximation of pitchingmoment using only body lift

            Flap_lift1 = flaplift_spline(M,Alpha1,-body_pitchingmoment1);% first approximation of flap lift

            total_lift1 = Cl1*A*q + Flap_lift1 + Thrust*sin(deg2rad(Alpha1)); %first total lift force, with normalised dynamic pressure, this needs to iterate to equal the original liftq




            Alpha2 = 15; %first guesses of AoA
            %Fuel Cost ===========================================================================

            Fueldt = FuelF_spline(M,Alpha2).*Efficiency;

            Isp = ThrustF_spline(M,Alpha2)./FuelF_spline(M,Alpha2); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================

            Cl2 = Cl_spline(M,Alpha2);

            body_pitchingmoment2 = pitchingmoment_spline(M, Alpha2);% first approximation of pitchingmoment using only body lift

            Flap_lift2 = flaplift_spline(M,Alpha2,-body_pitchingmoment2);% first approximation of flap lift

            total_lift2 = Cl2*A*q + Flap_lift2 + Thrust*sin(deg2rad(Alpha2));



            Alpha3 = Alpha2 - (-1+sqrt(5))/2*(Alpha2-Alpha1); %first golden section point

            Alpha4 = Alpha1 + (-1+sqrt(5))/2*(Alpha2-Alpha1); %first guesses of AoA


            while abs(Alpha4 - Alpha3) > 0.00005

            %Fuel Cost ===========================================================================

            Fueldt = FuelF_spline(M,Alpha3).*Efficiency;

            Isp = ThrustF_spline(M,Alpha3)./FuelF_spline(M,Alpha3); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================
            Cl3 = Cl_spline(M,Alpha3);

            body_pitchingmoment3 = pitchingmoment_spline(M, Alpha3);% first approximation of pitchingmoment using only body lift

            Flap_lift3 = flaplift_spline(M,Alpha3,-body_pitchingmoment3);% first approximation of flap lift

            total_lift3 = Cl3*A*q + Flap_lift3 + Thrust*sin(deg2rad(Alpha3));


            %Fuel Cost ===========================================================================

            Fueldt = FuelF_spline(M,Alpha4).*Efficiency;

            Isp = ThrustF_spline(M,Alpha4)./FuelF_spline(M,Alpha4); % this isnt quite Isp (doesnt have g) but doesnt matter

            Thrust = Isp.*Fueldt; % Thrust (N)
            %======================================================================
            Cl4 = Cl_spline(M,Alpha4);

            body_pitchingmoment4 = pitchingmoment_spline(M, Alpha4);% first approximation of pitchingmoment using only body lift

            Flap_lift4 = flaplift_spline(M,Alpha4,-body_pitchingmoment4);% first approximation of flap lift

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

            flapdeflection = flapdeflection_spline(M,Alpha4,-body_pitchingmoment4);

            Drag = Cd_spline(M,Alpha4)*A*q +  flapdrag_spline(M,Alpha4,-body_pitchingmoment4);
%               Drag = Cd_spline(M,Alpha4)*A*q ; % changed to just body drag

            liftarray(end,4) = Alpha4;

            liftarray(end,5) = flapdeflection;

            liftarray(end,6) = Drag;
            
            liftarray(end,7) = -body_pitchingmoment4;

        end
    end
end

% create splines
% given M and lift force / q , find AoA, flap deflection and total drag
% force / q
AoA_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,4)); 
flapdeflection_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,5));
Drag_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,6));
Flap_pitchingmoment_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3),liftarray(:,7));

end