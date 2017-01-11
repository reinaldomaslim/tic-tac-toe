clc;
clear;
arduino = serial('COM3', 'BaudRate', 9600);
fopen(arduino);
y=zeros(1,9);
for i=1:9
    
    y(i)=fscanf(arduino, '%d');
end
fclose(arduino);