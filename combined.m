%initiation
b=zeros(3,3);
turn=ceil(rand*2);
gaSt=0;
hArmController = ArmController(1:5,1);
turn=1;
%main game loop
while gaSt<9
    %check winning
     win=0;
    for i=1:2
        if b(1)==i && b(2)==i && b(3)==i
            win=i;
        elseif b(4)==i && b(5)==i && b(6)==i
            win=i;
        elseif b(7)==i && b(8)==i && b(9)==i
            win=i;
        elseif b(1)==i && b(4)==i && b(7)==i
            win=i;
        elseif b(2)==i && b(5)==i && b(8)==i
            win=i;
        elseif b(3)==i && b(6)==i && b(9)==i
            win=i;
        elseif b(1)==i && b(5)==i && b(9)==i
            win=i;
        elseif b(3)==i && b(5)==i && b(7)==i
            win=i;
        end
    end
    if win~=0
        break;
    end
    %finish checking winning
    
    %movement from either side
    if turn==1
        num=input('pls show your move:');
        %%input should come from arduino##
        b(num)=1;
        gaSt=gaSt+1;
        turn=2;
    elseif turn==2
        %strategy starts
        num=0;
        i=1;
        j=2;
        pause(0.5);

        while num==0
            if i==1     
                s=[1 2 3];
            elseif i==2
                s=[4 5 6];
            elseif i==3
                s=[7 8 9];
            elseif i==4
                s=[1 4 7];
            elseif i==5
                s=[2 5 8];
            elseif i==6
                s=[3 6 9];
            elseif i==7
                s=[1 5 9];
            elseif i==8
                s=[3 5 7];
            elseif i==9 && j==2
                j=1;
                i=1;
            elseif i==9 && j==1
                while num==0
                    c=ceil(rand*9);
                    if b(c)==0
                        num=c;
                    end
                end
                %%arm should move here accordingly.num is the position input for the robotic arm##
         end
	
            if b(s(1))==j && b(s(2))==j && b(s(3))==0
                num=s(3);
            elseif b(s(1))==j && b(s(2))==0 && b(s(3))==j
                num=s(2);
            elseif b(s(1))==0 && b(s(2))==j && b(s(3))==j
                num=s(1);
            end
            i=i+1;
        end
            %strategy end
        hArmController.Pressbutton(num);
        b(num)=2;
        gaSt=gaSt+1;
        turn=1;
    end
disp(b); 
end

%if winning in the middle
if win==1
        disp('Human wins');
       %%blinkall the red lights##
       hArmController.Dance();
    elseif win==2
        disp('Robot Arm wins');
       %%blink all the blue lights##
       hArmController.Dance2();
       hArmController.Dance2();
end

%checking game in the end if not finish in the middle
if gaSt==9
    gaSt=0;
    win=0;
    for i=1:2
        if b(1)==i && b(2)==i && b(3)==i
            win=i;
        elseif b(4)==i && b(5)==i && b(6)==i
            win=i;
        elseif b(7)==i && b(8)==i && b(9)==i
            win=i;
        elseif b(1)==i && b(4)==i && b(7)==i
            win=i;
        elseif b(2)==i && b(5)==i && b(8)==i
            win=i;
        elseif b(3)==i && b(6)==i && b(9)==i
            win=i;
        elseif b(1)==i && b(5)==i && b(9)==i
            win=i;
        elseif b(3)==i && b(5)==i && b(7)==i
            win=i;
        end
    end
    if win==1
        disp('Human Wins');
       %%blink all the red lights##
       hArmController.Dance2();
       
    elseif win==2
        disp('Arm Robot Wins');
       %%blink all the blue lights##
       hArmController.Dance();
    elseif win==0
        disp('Draw');
        %%blink all the lights##
        hArmController.Dance();
        
    end
end

