/* sketch 3 
turn on a LED when the button is pressed and let it on 
until the button is pressed again
*/


int BLED[9] = {39, 37, 35, 33, 31, 29, 27, 25, 23};
int RLED[9]=  {38, 36, 34, 32, 30, 28, 26, 24, 22};
int pinButton[9]=  {2, 3, 4, 5, 6, 7, 8, 9, 10};
int resetButton=11;
int blockStatus=0;
int b[9]={0,0,0,0,0,0,0,0,0};
int j;



int stateButton[9]={LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
int previous[9] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW};

long time = 0;
long debounce = 200;
int counter = 0;
int win=0;
int matlabData;
int draw=0;
 
void setup() {
  Serial.begin(9600);
  int i;
  for(i=0;i<9;i++){
    pinMode(pinButton[i], INPUT);
    pinMode(resetButton, INPUT);
    pinMode(RLED[i], OUTPUT);
    pinMode(BLED[i], OUTPUT);
  }
}
 
void loop() {
  int i;
  
  for(i=0;i<9;i++){

    stateButton[i] = digitalRead(pinButton[i]);
    if(stateButton[i] == HIGH && previous[i] == LOW && ((millis() - time) > debounce)) {
  
     
      if(digitalRead(RLED[i])==HIGH||digitalRead(BLED[i])==HIGH){
        break;}
      counter++;
      time = millis();
       
      
      if(counter%2==0){
          digitalWrite(RLED[i], HIGH);
          draw++;
          blockStatus=i+1;
          b[i]=1;
      } 
      else {
          digitalWrite(BLED[i], HIGH);
          blockStatus=i+10;
          b[i]=2;
          draw++;
      }
    }
    
    Serial.write(blockStatus);
    
    previous[i] == stateButton[i];
    
    
    
    
  }

    if(digitalRead(resetButton)==HIGH){
      for(i=0;i<9;i++){
        digitalWrite(RLED[i], LOW);
        digitalWrite(BLED[i], LOW);
        blockStatus=0;
        counter = 0;
        b[i]=0;
        draw=0;
        }
    }
   
   for (j=0;j<3;j++){
        if (b[0]==j && b[1]==j && b[2]==j){
            win=j;}
        else if (b[3]==j && b[4]==j && b[5]==j){
            win=j;}
        else if (b[6]==j && b[7]==j && b[8]==j){
            win=j;}
        else if (b[0]==j && b[3]==j && b[6]==j){
            win=j;}
        else if (b[1]==j && b[4]==j && b[7]==j){
            win=j;}
        else if (b[2]==j && b[5]==j && b[8]==j){
            win=j;}
        else if (b[0]==j && b[4]==j && b[8]==j){
            win=j;}
        else if (b[2]==j && b[4]==j && b[6]==j){
            win=j;}
    } 
    
   if(draw==9 && win ==0) // if there is data to read
     {
      win=3;} // read data          
    // read data
    
   if(win==1)
      for(i=0;i<9;i++){
        delay(100);
        digitalWrite(BLED[i], LOW);
        digitalWrite(RLED[i], HIGH);
        delay(100);
        digitalWrite(RLED[i], LOW);
        delay(100);
        digitalWrite(RLED[i], HIGH);
     }                                               // red wins
   else if(win==2)
      for(i=0;i<9;i++){
        delay(100);
        digitalWrite(RLED[i], LOW);
        digitalWrite(BLED[i], HIGH);
        delay(100);
        digitalWrite(BLED[i], LOW);
        delay(100);
        digitalWrite(BLED[i], HIGH);
      }  // blue wins
    else if(win==3){
      for(i=0;i<9;i++){
        delay(100);
        digitalWrite(RLED[i], LOW);
        digitalWrite(BLED[i], LOW);
        delay(100);
        digitalWrite(RLED[i], HIGH);
        digitalWrite(BLED[i], HIGH);
        delay(100);
        digitalWrite(RLED[i], LOW);
        digitalWrite(BLED[i], LOW);
        delay(100);
        digitalWrite(RLED[i], HIGH);
        digitalWrite(BLED[i], HIGH);
      }   // turn light off 
    }
  
}
  
  



  

