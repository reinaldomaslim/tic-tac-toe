%clear all
%clc
 
a=serial('COM3','BaudRate',9600);
 
x=3;
fopen(a);
fprintf( a , '%d', 3);

fclose(a);
