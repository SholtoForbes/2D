function [AoA_spline, flapdeflection_spline, Dragq_spline] = LiftForceInterp(communicator,communicator_trim)
%Lift Force interpolator

%this module takes values from communicator and communicator-trim and finds
%appropriate AoA and flap deflection values to equalise a given q
%normalised lift force. this allows for splines to be created that only
%require M and lift force / q to give flap deflection, AoA and total drag


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


A = 60; % reference area

% golden sections method, to search for a variety of M and lift forces / q
%equalises the pitching moment of flap and body to calculate lift. works
%towards the correct AoA (and corresponding flap pitching moment)
liftarray = [];
for M = 6:1:12 % M
    for Liftq = 0:0.2:10 %Lift force / q

        liftarray(end+1,1) = M;
        liftarray(end,2) = Liftq;

        
%         Alpha = Alpha_spline(M, Liftq/A); % first approximation of alpha using only body lift

        Alpha1 = 0;
        
        Cl1 = Cl_spline(M,Alpha1);
        
        body_pitchingmoment1 = pitchingmoment_spline(M, Alpha1);% first approximation of pitchingmoment using only body lift
        
        Flap_lift1 = flaplift_spline(M,Alpha1,-body_pitchingmoment1);% first approximation of flap lift
        
        total_liftq1 = Cl1*A + Flap_lift1/50000; %first total lift force, with normalised dynamic pressure, this needs to iterate to equal the original liftq
        

        
        
        Alpha2 = 30; %first guesses of AoA
        
        Cl2 = Cl_spline(M,Alpha2);
        
        body_pitchingmoment2 = pitchingmoment_spline(M, Alpha2);% first approximation of pitchingmoment using only body lift
        
        Flap_lift2 = flaplift_spline(M,Alpha2,-body_pitchingmoment2);% first approximation of flap lift
        
        total_liftq2 = Cl2*A + Flap_lift2/50000;
        
        
        
        Alpha3 = Alpha2 - (-1+sqrt(5))/2*(Alpha2-Alpha1); %first golden section point
        
        Alpha4 = Alpha1 + (-1+sqrt(5))/2*(Alpha2-Alpha1); %first guesses of AoA

        
        while abs(Alpha4 - Alpha3) > 0.00005


        Cl3 = Cl_spline(M,Alpha3);
        
        body_pitchingmoment3 = pitchingmoment_spline(M, Alpha3);% first approximation of pitchingmoment using only body lift
        
        Flap_lift3 = flaplift_spline(M,Alpha3,-body_pitchingmoment3);% first approximation of flap lift
        
        total_liftq3 = Cl3*A + Flap_lift3/50000;

        
                
        Cl4 = Cl_spline(M,Alpha4);
        
        body_pitchingmoment4 = pitchingmoment_spline(M, Alpha4);% first approximation of pitchingmoment using only body lift
        
        Flap_lift4 = flaplift_spline(M,Alpha4,-body_pitchingmoment4);% first approximation of flap lift
        
        total_liftq4 = Cl4*A + Flap_lift4/50000;
        
        

            if abs(Liftq - total_liftq4) > abs(Liftq - total_liftq3)
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
        
        Dragq = Cd_spline(M,Alpha4)*A +  flapdrag_spline(M,Alpha4,-body_pitchingmoment4)/50000;
        
        
        liftarray(end,3) = Alpha4;
        
        liftarray(end,4) = flapdeflection;
        
        liftarray(end,5) = Dragq;
        

    end
end

% create splines
% given M and lift force / q , find AoA, flap deflection and total drag
% force / q
AoA_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,3)); 
flapdeflection_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,4));
Dragq_spline = scatteredInterpolant(liftarray(:,1),liftarray(:,2),liftarray(:,5));

end