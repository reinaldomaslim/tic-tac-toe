%function y=arduinocomm()
 

%try
    
answer=1; % this is where we'll store the user's answer

s=serial('COM3','BaudRate',9600); % create serial communication object on port COM3
 
fopen(s); % initiate arduino communication
s.ReadAsyncMode='continuous';

readasync(s);

%y=zeros(1,9);
   
%for i=1:9
%	y(i)=fscanf(s, '%d');
%end

y = fscanf(s, '%d');


%pause(1);

%return;

%catch
    
%stopasync(s);

%fclose(s); % end communication with arduino

%delete(s);
%clear(s);

%return;

%end
