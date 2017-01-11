        %% function to calculate the rotating angle according to the given distance of the tip
        % correct?
        % input:
        % output:
        %angle[1;5]
        %distance[x,y,z]
        %param[d, k] offset distance of x and y, respectively.
        
        function angle = InverseKine(self, distance, param)
            
            angle=zeros(5,1);

            theta12= atan((param(2)+distance(2)+self.LinkLengths(5))/(distance(1)+param(1)));
            
            angle(3)=-acos((distance(1)+param(1))/(2*self.LinkLengths(2)*cos(theta12))-0.5);
            
            angle(4)=angle(3);
            
            angle(2)=theta12-angle(3);
            
            angle(5)=-pi/2-angle(2)-2*angle(3);
            
            angle(1)=atan(distance(3)/(distance(1)+param(1)));
            
            angle = self.HomeAngle + self.Direction * rotatingAngle;
            
        end