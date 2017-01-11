function [s,flag]= setupSerial(comPort)
flag=1;
s=serial(comPort);
set(s, 'DataBits', 8);
set(s, 'StopBits', 1);
set(s, 'BaudRate', 9600);
set(s, 'Parity', 'none');

fopen(s);

data= zeros(1,9);

for i=1:9
    data(1,i)=fscanf(s);
    
    if i==9
        i=1;
        fprintf( '%d ', data );
    end
    
end

fclose(s);


end
