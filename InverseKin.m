function angle = InverseKin(length, distance, param)
            
            angle=zeros(1,5);

            theta12= atan((param(2)+distance(2)+length(5))/(distance(1)+param(1)));
            
            angle(3)=-acos((distance(1)+param(1))/(2*length(2)*cos(theta12))-0.5);
            
            angle(4)=angle(3);
            
            angle(2)=theta12-angle(3);
            
            angle(5)=-pi/2-angle(2)-2*angle(3);
            
            angle(1)=-atan(distance(3)/(distance(1)+param(1)));
            
         
            angle=rad2deg(angle);
            
            %angle = self.HomeAngle + self.Direction * rotatingAngle;
            
        end