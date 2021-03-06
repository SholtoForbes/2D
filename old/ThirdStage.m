clear all

AoA_list = [];
        
theta_list = [];

Alt_list = [];



% temp = 1;
% for i1 = 27000.:1000.:45000.
%     Alt = i1;
%     
%     for i2 = 0:.5:10.
%         theta = i2; 
% 
%         propellant_list = [];
% %         temp2 = 1;
%         
%         %This finds the correct AoA
%         for i3 = 0.0:.5:20.0

% for i1 = 39000.:1000.:40000.
    
    i1 = 40000;
    Alt = i1;
    
    for i2 = 0:.2:15.
        theta = i2;  

        propellant_list = [];
%         temp2 = 1;
        
        %This finds the correct AoA
        for i3 = 0.0:.1:20.0  
            AoA = i3;



            delete cond.dat
            delete TRAJ3.ASC
            delete TRAJ3.BIN


            % edit CADACIN3.ASC
            C = fopen('CADIN3.ASC','w');
            cond = fopen('cond.dat','w');





            % AoA = 10.00;

            Cstring = ['IN300.ASC:2 run02\r\n' ...
            '01 OUTPUT 2,3        0003\r\n' ...
            '01 STAGE 2,3         0004\r\n' ...
            '02 G2  ENVIRONMENT   0023\r\n' ...
            '02 A1  AERODYNAMICS  0002\r\n' ...
            '02 A2  PROPULSION    0003\r\n' ...
            '02 A3  FORCES        0004\r\n' ...
            '02 D1  NEWTONS LAW   0017\r\n' ...
            '03 OPTMET            0053      1.0000\r\n' ...
            '03 OPNORO            0059      1.0000\r\n' ...
            '04*** PROPULSION *\r\n' ...
            '04BURN TIME=1000.0S\r\n' ...
            '03 MPROP             1300    1 0.0000\r\n' ...
            '03 FUELI             1302      2447.3\r\n' ...
            '03 FUELR             1303      14.710\r\n' ...
            '03 SPI               1304      350.00\r\n' ...
            '03 AEXIT             1316      0.6360\r\n' ...
            '03 THRTL             1310      1.0000\r\n' ...
            '04*** VEHICLE DATA\r\n' ...
            '03 MAERO             1200    1 11.000\r\n' ...
            '03 SREF              1401       0.870\r\n' ...
            '03 VMASSI            1305       2850.1\r\n' ...
            '03 ALPHAX            1203      ' num2str(AoA,'%10.4e') ' \r\n' ...
            '03 PHIMVX            1402      0.0000\r\n' ...
            '04*** LAUNCH CONDI\r\n' ...
            '03 BLON              1604        2.4874\r\n' ...
            '03 BLAT              1605       -0.1315\r\n' ...
            '03 BALT              1606       ' num2str(Alt,'%10.4e') ' \r\n' ...
            '03 DVBE              1613       2839.59\r\n' ...
            '03 PSIVGX            1602       -6.900\r\n' ...
            '03 THTVGX            1603       ' num2str(theta,'%10.4e') '\r\n' ...
            '04*** PRINTING AND\r\n' ...
            '03 PPP               2005      0.10000\r\n' ...
            '03 CPP               2015      1.0000\r\n' ...
            '03 DER               2664      0.01000\r\n' ...
            '10                      1\r\n' ...
            'VMASS  1309    0            0.0\r\n' ...
            '06\r\n' ...
            '13']
            fprintf(C,Cstring);


            datString = ['2nd stage start: DVBE,BALT for M6 flight\r\n' ...
                '    1797.9, 26665.\r\n' ...
                '    DER\r\n' ...
                '    0.01\r\n' ...
                '    alpha_guess thtvgx_guess\r\n' ...
                '    3.0         1.8	\r\n' ...
                '    3rd STAGE start:	\r\n' ...
                '    wetmass     propmass    HSmass  Sref	EngineMass\r\n' ...
                '    2850.16     1787.75     302.75  0.87    100.00\r\n' ...
                '    fuelrate    Spi       Nozzle exit area (r=.45m)\r\n' ...
                '    14.71       350.0     0.636\r\n' ...
                '    DVBE      BALT      FPA      BLON    BLAT     PSIVGX\r\n' ...
                '    2839.59   32599.9   0.2233   2.4874  -0.1315  -6.8997\r\n' ...
                '3500']

            fprintf(cond,datString);


%             !CADAC3.exe
            system(['CADAC3.exe']);

            cond2 = txt2mat('cond.dat'); % for whatever reason this only reads the last line (the only one necessary)

            cond2


%             propellant_list(temp2) = cond2(3);
%             
%           
%             temp2 = temp2 + 1;

            propellant_list(end+1) = cond2(3);
            
            fclose('all');

            

        end
        
        [propellant_m,I] = min(propellant_list);

        minpropellant_AoA = I*0.1 - 0.1; % as each step is 0.1 degrees, starting from 0


        
%         AoA_list(temp) = minpropellant_AoA;
%         
%         theta_list(temp) = theta;
%         
%         Alt_list(temp) = Alt;
%         
%         Propellant_list(temp) = minpropellant_AoA;
% 
%         temp = temp + 1;



        AoA_list(end+1) = minpropellant_AoA;
        
        theta_list(end+1) = theta;
        
        Alt_list(end+1) = Alt;
        
        propellant_list(end+1) = propellant_m;
        
        thirdstage = fopen('thirdstage.dat','a+');
        
        thirdstage_results = [num2str(theta,'%10.4e') ' ' num2str(Alt,'%10.4e') ' ' num2str(propellant_m,'%10.4e') ' ' num2str(minpropellant_AoA,'%10.4e') '\r\n'] ;
        
        fprintf(thirdstage,thirdstage_results);
        
        close all
    end
% end

fclose('all');