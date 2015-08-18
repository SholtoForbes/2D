%2ndStageEngineInterpolator

% this runs the file EngineData.exe which takes input of Mach, T and p and
% produces output of thrust force (N) and fuel usage (kg/s).

delete input
delete output
delete engineoutput_matrix

Atmosphere = dlmread('atmosphere.txt'); % import data from atmosphere matrix

wcap = 0.65;

for i = 26000:1000:40000
    V = i;

    for j = 6:1.:13
        M = j;

        T = spline( Atmosphere(:,1),  Atmosphere(:,2), V); % Calculate speed of sound using atmospheric data

        p = spline( Atmosphere(:,1),  Atmosphere(:,3), V); % Calculate density using atmospheric data

        input = fopen('input','w');

        inputstring = [num2str(M,'%10.4e') ' ' num2str(T,'%10.4e') ' ' num2str(p,'%10.4e') ' ' num2str(wcap,'%10.4e')];

        fprintf(input,inputstring);

        fclose('all');

        system(['EngineData.exe']);
        % !EngineData.exe

        output_temp = dlmread('output');

        engineoutput_matrix = fopen('engineoutput_matrix','a+');

        outputstring = [num2str(M,'%10.4e') ' ' num2str(V,'%10.4e') ' ' num2str(output_temp(1),'%10.4e') ' ' num2str(output_temp(2),'%10.4e') '\r\n'];

        fprintf(engineoutput_matrix,outputstring);

        fclose('all');
    end
end
