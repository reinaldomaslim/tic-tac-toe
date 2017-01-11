clc;
clear;
arduino = serial('COM3', 'BaudRate', 9600);
fopen(arduino);
x=linspace(1,100);
for i=1:length(x)
    y(i)=fscanf(arduino, '%d');
end
fclose(arduino);