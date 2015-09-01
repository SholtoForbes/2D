%Lift Force interpolator

communicator = importdata('communicatornew.txt');

communicator_trim = importdata('communicator_trim.txt');

Cl_spline = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,3)); %find cl given M, AoA
flaplift_spline_2 = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,3),communicator_trim(:,6)); %find flap lift, given M, AoA, flap deflection

Cd_spline = scatteredInterpolant(communicator(:,1),communicator(:,3),communicator(:,4)); % find Cd given M, Cl 
Alpha_spline = scatteredInterpolant(communicator(:,1),communicator(:,3),communicator(:,2)); % find AoA given M, Cl
 
pitchingmoment_spline = scatteredInterpolant(communicator(:,1),communicator(:,2),communicator(:,11)); % find pitching moment given M, AoA changed from main code
% pitchingmoment_spline = scatteredInterpolant(communicator(:,1),communicator(:,3),communicator(:,6));
flapdeflection_spline = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,3));

flapdrag_spline = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,5));

flaplift_spline = scatteredInterpolant(communicator_trim(:,1),communicator_trim(:,2),communicator_trim(:,4),communicator_trim(:,6));


A = 60;


liftarray = [];
for M = 6:1:12 % M
    for Liftq = 0:0.2:10 %Lift force / q

        liftarray(end+1,1) = M;
        liftarray(end,2) = Liftq;

        
%         Alpha = Alpha_spline(M, Liftq/A); % first approximation of alpha using only body lift

        Alpha1 = 0;
        
        Cl = Cl_spline(M,Alpha1);
        
        body_pitchingmoment = pitchingmoment_spline(M, Alpha1);% first approximation of pitchingmoment using only body lift
        
        Flap_lift = flaplift_spline(M,Alpha1,-body_pitchingmoment);% first approximation of flap lift
        
        total_liftq = Cl*A + Flap_lift/50000; %first total lift force, with normalised dynamic pressure, this needs to iterate to equal the original liftq
        
%COPY THIS INTO SECOND APPROX, ADN THEN INTO A SEARCH ROUTINE FOR BEST AOA
        
        
        Alpha2 = 10; %first guesses of AoA
        
%         body_pitchingmoment = pitchingmoment_spline(M, Liftq/A);% first approximation of pitchingmoment using only body lift
%         
%         Flap_lift = flaplift_spline(M,Alpha,-body_pitchingmoment);% first approximation of flap lift
%         
%         total_liftq = Liftq + Flap_lift/50000; %first total lift force, with normalised dynamic pressure, this needs to iterate to equal the original liftq
        
        
        
        while abs(Liftq - total_liftq) > 0.05
            
%             Alpha = Alpha_spline(M, Liftq/A); % first approximation of alpha using only body lift
%         
%             body_pitchingmoment = pitchingmoment_spline(M, Liftq/A);% first approximation of pitchingmoment using only body lift
%         
%             Flap_lift = flaplift_spline(M,Alpha,-body_pitchingmoment);% first approximation of flap lift
%         
%             total_liftq = Liftq + Flap_lift/50000 %total lift force, with normalised dynamic pressure, this needs to iterate to equal the original liftq
        
        end
        
        flapdeflection = flapdeflection_spline(M,Alpha,-body_pitchingmoment);
        
        liftarray(end,3) = Alpha;
        
        liftarray(end,4) = flapdeflection;
        

    end
end