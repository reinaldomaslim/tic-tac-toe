BLED = ['D39'; 'D37'; 'D35'; 'D33'; 'D31'; 'D29'; 'D27'; 'D25'; 'D23'];
RLED=  ['D38'; 'D36'; 'D34'; 'D32'; 'D30'; 'D28'; 'D26'; 'D24'; 'D22'];
pinButton=  ['D2 '; 'D3 '; 'D4 '; 'D5 '; 'D6 '; 'D7 '; 'D8 ';'D9 '; 'D10'];
resetButton='D11';
RLEDStatus=zeros(1,9);
BLEDStatus=zeros(1,9);

stateButton=zeros(1,9);
previous=zeros(1,9);
counter=0;



while(1)
  
  for i=1:9
      
    stateButton(i) = readDigitalPin(a, pinButton(i,:));
    
    if stateButton(i) == 1
     
      if(RLEDStatus(i)==1||BLEDStatus(i)==1)
        break;
      end
        
      counter=counter+1;
      
       
      
      if mod(counter,2)==0
          writeDigitalPin(a, RLED(i,:), 1);
          RLEDStatus(i)=1;
      else 
          writeDigitalPin(a, BLED(i,:), 1);
          BLEDStatus(i)=1;
      end
   
    end

    if readDigitalPin(a, resetButton)==1
      for j=1:9
        writeDigitalPin(a, RLED(j,:), 0);
        writeDigitalPin(a, BLED(j,:), 0);
        blockStatus=0;
        counter = 0;
        RLEDStatus(i)=0;
        BLEDStatus(i)=0;
      end
    end
   
    
  end
  
  
  
  
  
  
  
end
  
  



  
