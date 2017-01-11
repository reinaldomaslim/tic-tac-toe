classdef ArmController < handle
    
    
    properties
        
    end
    
    
    properties(SetAccess=protected)
        
        % default connecting port
        DEFAULT_PORTNUM = 4; % com4
        % default baud number
        DEFAULT_BAUDNUM = 1; % 1mbps
        
        % IDs of dynamixels
        IDs = [];
        
        % number of dynamixels
        NumberDxls = 5;
        
        % length of links (l1 = 86)
        LinkLengths = [0, 67, 67, 67, 45];
        
        % moving range of each joint (binary value)
        % Row 1: CCW; Row 2: CW      Col 1 to 5: joint 1 to 5
        % JointRange = [860,860,820,860;165,155,155,200];
        JointRange = [1024,860,860,860,860;0,155,155,155,155];
       
        % moving mode: joint mode (pricise position control) or wheel mode
        % (roughly torque control)
        Mode = 1;   % joint mode in default
        
        % home angle (degree)
        HomeAngle = [0,-90,0,0,0] % +[0,90,0,-90,-90];
        
        % default speed (rpm)
        DefSpeed = 30;
        MaxSpeed = 59;
        
        % moving direction of each dynamixel for parallel motion
        Direction = [-1,1,1,1,1];
        
        % control table address
        P_ID                    = 3;   
        P_GOAL_POSITION         = 30;
        P_MOVING_SPEED          = 32;
        P_PRESENT_POSITION      = 36;
        P_LOAD_DIRECTION        = 41;
        P_MOVING                = 46;
        BROADCAST_ID            = 254;
        P_CW_ANGLE_LIMIT        = 6;
        P_CCW_ANGLE_LIMIT       = 8;
        P_CW_COMPLIANCE_MARGIN  = 26;
        P_CCW_COMPLIANCE_MARGIN = 27;
        P_CW_COMPLIANCE_SLOPE   = 28;
        P_CCW_COMPLIANCE_SLOPE  = 29;
        
        % specification of AX-12A
        ANGLE_RESOLUTION   = 1024;
        ANGLE_RANGE        = 300;
        AmpPos             = 512; % initialization position in factory setting
        
        % Error bit
        ERRBIT_VOLTAGE     = 1;
        ERRBIT_ANGLE       = 2;
        ERRBIT_OVERHEAT    = 4;
        ERRBIT_RANGE       = 8;
        ERRBIT_CHECKSUM    = 16;
        ERRBIT_OVERLOAD    = 32;
        ERRBIT_INSTRUCTION = 64;
        
        % Communication result
        COMM_TXSUCCESS     = 0;
        COMM_RXSUCCESS     = 1;
        COMM_TXFAIL        = 2;
        COMM_RXFAIL        = 3;
        COMM_TXERROR       = 4;
        COMM_RXWAITING     = 5;
        COMM_RXTIMEOUT     = 6;
        COMM_RXCORRUPT     = 7;
        
    end
    
    
    methods
        
        
        %% default contructor
        % input:
        % output:
        function self = ArmController(IDs,mode)
            
            if ~isempty(IDs)
                if length(IDs) ~= 5
                    error('Argument should be a 1*5 ID vector!');
                else
                    self.IDs = IDs;
                end
            end
            
            self.Mode = mode;
            
            self.Initilize();
            
        end
        
        
        %% function to initilize the arm
        % input:
        % output:
        function Initilize(self)
            
            % load the dynamixel library
            % Note: APIs can be used only after loading the dynamixel library!
            loadlibrary('dynamixel','dynamixel.h');
            
            % list all the functions could be used
%             libfunctions('dynamixel');
            
            % initilize the USB2Dyanmixel and makes ready status
            res = calllib('dynamixel','dxl_initialize',self.DEFAULT_PORTNUM,self.DEFAULT_BAUDNUM);
            
            if res == 1
                
                disp('Succeed to open USB2Dynamixel!');
                
                % check the status of dynamixels
                NumConnectedDxl = 0;
                for i = 1:self.NumberDxls
                    calllib('dynamixel','dxl_ping',self.IDs(i));
                    if int32(calllib('dynamixel','dxl_get_result')) == self.COMM_RXSUCCESS
                        NumConnectedDxl = NumConnectedDxl + 1;
                    end
                end
                
                if NumConnectedDxl == self.NumberDxls
                    % all the dynamixels are already connected to USB2Dyanmixel
                    
                    % initilize the gripper to the correct moving mode
                    self.ChangeDxlMode(self.Mode);
                    
                    % get the gripper ready: start from home position
                    self.Home();
                    
                    disp('Initilize Nextage arm sucessfully!');
                    
                else
                    disp('Failed to communicate with dynamixels! To do: 1) Check the power supply; 2) Check the IDs; 3) Check the connection.');
                end
                
            else
                
                disp('Failed to open USB2Dynamixel!');
                
            end
                     
        end   
        
        
        %% function to terminate the gripper
        % must run this function after usage
        % input:
        % output:
        function Terminate(self)
            
            % ternimate the USB2Dynamixel, otherwise other devices can not
            % access it
            calllib('dynamixel','dxl_terminate');
            
            % unload the dynamixel library
            unloadlibrary('dynamixel');
            
        end
                %% function to press button
        % must run this function after usage
        % input:
        % output:
        function Pressbutton(self, BlockID)
            param=[62 -18];
            
        switch BlockID
            case 1
                distance=[126 0 -50];
            case 4
                distance=[120 0 0];
            case 7
                distance=[126 0 50];
            case 2
                distance=[85 0 -50];
            case 5
                distance=[75 0 0];
            case 8
                distance=[85 0 50];
            case 3
                distance=[40 0 -55];
            case 6
                distance=[35 0 -10];
            case 9
                distance=[40 0 55];
        end
        
        self.Home();
        pause(1);
        self.MoveParallelly(distance+[0,40,0], param, 25);
        pause(1);
        self.MoveParallelly(distance+[5,-35,0], param, 25);
        pause(1);
        self.MoveParallelly(distance+[0,10,0], param, 25);
        pause(1);
        self.Home();
        pause(1);
        end
        
                %% function to press button
        % must run this function after usage
        % input:
        % output:
        function MoveAbove(self, BlockID)
            param=[62 -18];
            
        switch BlockID
            case 1
                distance=[120 10 -50];
            case 4
                distance=[120 -10 0];
            case 7
                distance=[126 0 50];
            case 2
                distance=[75 0 0];
            case 5
                distance=[75 10 0];
            case 8
                distance=[85 0 50];
            case 3
                distance=[40 0 -90];
            case 6
                distance=[35 0 0];
            case 9
                distance=[40 0 90];
        end
        
        
        
        self.MoveParallelly(distance+[0,70,0], param, 25);
        pause(0.2);
        
        end
        %% function to change the ID of a dynamixel
        % input: NoDxl - indicate which dynamixel to be changed, whos value is
        % the index of the self.IDs vector
        %        ID - the ID
        % output:
        function ChangeID(self,NoDxl,ID)
            
            calllib('dynamixel','dxl_write_byte',self.IDs(NoDxl),self.P_ID,ID);
            self.CheckError();
            
            % update the ID table stored in the object
            self.IDs(NoDxl) = ID;
            
        end
        
       
        
        
        %% function to release the gripper
        % Always works in the joint mode for easy control
        % input:
        % output:
        function Home(self)
            
            
            rotatingAngle=[0,90,0,-90,-90];
             % calculate the accurate rotating angle 
            angle =self.HomeAngle + rotatingAngle;
            
            
            speed = self.DefSpeed;
            
            
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
           
            
        end
         %% function to control the distance between the two finger tips
        % input:
        %        speed - moving speed (unit: rpm, maximum 59)
        % output:
        function MoveParallelly(self,distance,param,speed)
            
            % calculate the accurate rotating angle 
            angle = self.InverseKine(distance,param);
            
            if isempty(speed)
                speed = self.DefSpeed;
            end
            
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
        end
        
        
                %% function to control the distance between the two finger tips
        % input:
        %        speed - moving speed (unit: rpm, maximum 59)
        % output:
        function Dance2(self)
            
            speed=0;
            pause(0.2)
            angle = self.HomeAngle+[0, 180, -90, -90, -90];
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
            pause(0.2);
            
             angle = self.HomeAngle+[0, 90, 0, 0, 0];
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
            pause(0.4);
          
            
             angle = self.HomeAngle+[0, 180, -90, -90, -90];
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
            pause(0.2);
        end
       
        
                %% function to control the distance between the two finger tips
        % input:
        %        speed - moving speed (unit: rpm, maximum 59)
        % output:
        function Dance(self)
            
            speed=0;
            
            angle = self.HomeAngle+[0, 180, -90, -90, -90];
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
            pause(1);
            
            self.MoveAbove(9);
            self.MoveAbove(6);
            self.MoveAbove(3);
            self.MoveAbove(6);
            self.MoveAbove(9);
            
            self.MoveAbove(9);
            self.MoveAbove(6);
            self.MoveAbove(3);
            self.MoveAbove(6);
            self.MoveAbove(9);
            
             angle = self.HomeAngle+[0, 90, 0, -90, -90];
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
            pause(0.2);
            
             angle = self.HomeAngle+[0, 180, -90, -90, -90];
            % get the gripper ready: start from home position
            self.SyncWritePositionSpeed(self.IDs,angle,speed);
            
        end
        
        
        
        
                %% function to calculate the rotating angle according to the given distance of the tip
        % correct?
        % input:
        % output:
        %angle[1;5]
        %distance[x,y,z]
        %param[d, k] offset distance of x and y, respectively.
        
        function angle = InverseKine(self, distance, param)
            
            angle=zeros(1,5);

            theta23= atan((param(2)+distance(2)+self.LinkLengths(5))/(distance(1)+param(1)));
            
            angle(3)=-acos((distance(1)+param(1))/(2*self.LinkLengths(2)*cos(theta23))-0.5);
            
            angle(4)=angle(3);
            
            angle(2)=theta23-angle(3);
            
            angle(5)=-pi/2-angle(2)-2*angle(3);
            
            angle(1)=-atan(distance(3)/(distance(1)+param(1))); %-ve because rotating direction 
            
            
            
            rotatingAngle = rad2deg(angle);
            
            angle = self.HomeAngle + rotatingAngle;
            
            
            
        end
        
        
        %% function to implement parallel movement of finger tips by giving rotating angle
        % input: angle - rotating angle of each dynamixel with respect to the home position (unit: degree)
        % output:
        function MoveParallellyByAngle(self,angle)
            
            % calculate the absolute angle for each dynamixel
            goalAngle = self.HomeAngle + self.Direction * angle;
            
            % move
            self.SyncWritePosition(self.IDs,goalAngle);
            
        end
        
        
        %% function to write goal positions to multiple dynamixels simultaneously
        % input:
        % ouput:
        function SyncWritePosition(self,IDs,angle)
            
            position = self.Angle2Position(angle);
            
            calllib('dynamixel','dxl_set_txpacket_id',self.BROADCAST_ID);
            % INST_SYNC_WRITE = 131
            calllib('dynamixel','dxl_set_txpacket_instruction',131);
            calllib('dynamixel','dxl_set_txpacket_parameter',0,self.P_GOAL_POSITION);
            calllib('dynamixel','dxl_set_txpacket_parameter',1,2);
            
            % number of dynamixels to move
            NoDxl2Move = length(IDs);
            for i = 0:1:NoDxl2Move-1
                % ID
                calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i,IDs(i+1));
                % lowbyte of position
                calllib('dynamixel','dxl_set_txpacket_parameter',3+3*i, calllib('dynamixel','dxl_get_lowbyte',position(i+1)));
                % highbyte of position
                calllib('dynamixel','dxl_set_txpacket_parameter',4+3*i, calllib('dynamixel','dxl_get_highbyte',position(i+1)));
            end
            % Length = (L+1)*NoDxl2Move+4
            calllib('dynamixel','dxl_set_txpacket_length',3*NoDxl2Move+4);
            calllib('dynamixel','dxl_txrx_packet');
            self.CheckError();
            
        end
        
        
        %% function to write moving speeds to multiple dynamixels simultaneously
        % input: IDs - IDs of dynamixels to be driven
        %        speed - target speed (rpm)
        % output:
        function SyncWriteSpeed(self,IDs,speed)
            
            speed = self.RPM2BinSpeed(speed);
            
            calllib('dynamixel','dxl_set_txpacket_id',self.BROADCAST_ID);
            % INST_SYNC_WRITE = 131
            calllib('dynamixel','dxl_set_txpacket_instruction',131);
            calllib('dynamixel','dxl_set_txpacket_parameter',0,self.P_MOVING_SPEED);
            calllib('dynamixel','dxl_set_txpacket_parameter',1,2);
            
            % number of dynamixels to move
            NoDxl2Move = length(IDs);
            for i = 0:1:NoDxl2Move-1
                calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i,IDs(i+1));
                calllib('dynamixel','dxl_set_txpacket_parameter',3+3*i, calllib('dynamixel','dxl_get_lowbyte',speed(i+1)));
                calllib('dynamixel','dxl_set_txpacket_parameter',4+3*i, calllib('dynamixel','dxl_get_highbyte',speed(i+1)));
            end
            % Length = (L+1)*NoDxl2Move+4
            calllib('dynamixel','dxl_set_txpacket_length',3*NoDxl2Move+4);
            calllib('dynamixel','dxl_txrx_packet');
            self.CheckError();
            
        end
        
        
        %% function to write goal positions and moving speeds to multiple dynamixels simultaneously
        % input:
        % output:
        function SyncWritePositionSpeed(self,IDs,angle,speed)
            
            % preprocess
            position = self.Angle2Position(angle);
            %speed = self.RPM2BinSpeed(speed);
            speed=240;
            calllib('dynamixel','dxl_set_txpacket_id',self.BROADCAST_ID);
            % INST_SYNC_WRITE = 131
            calllib('dynamixel','dxl_set_txpacket_instruction',131);
            calllib('dynamixel','dxl_set_txpacket_parameter',0,self.P_GOAL_POSITION);
            calllib('dynamixel','dxl_set_txpacket_parameter',1,4);
            
            % number of dynamixels to move
            NoDxl2Move = length(IDs);
            
            for i = 1:1:NoDxl2Move
                calllib('dynamixel','dxl_set_txpacket_parameter',2+5*(i-1),IDs(i));
                % 2+5*i+1 = 3+5i
                calllib('dynamixel','dxl_set_txpacket_parameter',3+5*(i-1), calllib('dynamixel','dxl_get_lowbyte',position(i)));
                % 2+5*i+2 = 4+5i
                calllib('dynamixel','dxl_set_txpacket_parameter',4+5*(i-1), calllib('dynamixel','dxl_get_highbyte',position(i)));
                % 2+5*i+3 = 5+5i
                %calllib('dynamixel','dxl_write_word',i ,32, 400);
                calllib('dynamixel','dxl_set_txpacket_parameter',5+5*(i-1), calllib('dynamixel','dxl_get_lowbyte',speed));
                % 2+5*i+4 = 6+5i
                calllib('dynamixel','dxl_set_txpacket_parameter',6+5*(i-1), calllib('dynamixel','dxl_get_highbyte',speed));
            end
            % Length = (L+1)*NoDxl2Move+4
            calllib('dynamixel','dxl_set_txpacket_length',5*NoDxl2Move+4);
            calllib('dynamixel','dxl_txrx_packet');
            self.CheckError();
            
        end
        
        
        %% function to read present positions simultaneously
        % input:
        % output:
        function angle = SyncReadPosition(self)
            
            angle = zeros(1,5);
            
            for i = 1:self.NumberDxls
                angle(i) = int32(self.Position2Angle(calllib('dynamixel','dxl_read_word',self.IDs(i),self.P_PRESENT_POSITION)));
            end
            
        end
        
        
        %% function to detect the load direction
        % input:
        % output:
        function loaddir = DetectLoadDir(self)
            
            loaddir = '00000';
            
            for i = 1:self.NumberDxls
                
                loaddir(i) = int2str(int32(calllib('dynamixel','dxl_read_byte',self.IDs(i),self.P_LOAD_DIRECTION))~=0);
                
            end
                        
        end
        
        

        %% function to change the moving mode of dynamixels
        % by seting the CCW_ANGLE_LIMIT and CW_ANGLE_LIMIT
        % input:
        % output:
        function ChangeDxlMode(self,mode)
            
            if self.Mode == mode
                
                % no change
                return;
                
            else
                
                % stop moving by seting speed to '0'
                % self.SyncWriteSpeed(zeros(1,4));
                
                % set the CCW_ANGLE_LIMIT according to the 'mode'
                % supposing CW_ANGLE_LIMIT is '0'
                if mode
                    % joint mode
                    CCW_ANGLE_LIMIT = self.JointRange(1,:);
                    CW_ANGLE_LIMIT = self.JointRange(2,:);
                else
                    % wheel mode
                    % set the speeds to be zero first,otherwise the motor
                    % may rotate afther changing to wheel mode
                    self.SyncWriteSpeed(self.IDs,zeros(1,5));
                    CCW_ANGLE_LIMIT = zeros(1,5);
                    CW_ANGLE_LIMIT = CCW_ANGLE_LIMIT;
                end
                
                for i = 1:self.NumberDxls
                    
                    calllib('dynamixel','dxl_write_word',self.IDs(i),self.P_CCW_ANGLE_LIMIT,CCW_ANGLE_LIMIT(i));
                    calllib('dynamixel','dxl_write_word',self.IDs(i),self.P_CW_ANGLE_LIMIT,CW_ANGLE_LIMIT(i));
                    
                end
                
                % update the control mode stored in the object
                self.Mode = mode;
                
            end
            
        end
        
        
        %% function to delay until dynamixels finish motion
        % input: No
        % output: No
        function WaitUntilFinish(self)
                       
            % P_MOVING -> 1 = dynamixel is moving; otherwise stops.
            
            while(1)
                waitingflag = 0;
                pause(0.1);
                for id = self.IDs
                    moving = int32(calllib('dynamixel','dxl_read_byte',id,self.P_MOVING));
                    waitingflag = waitingflag + moving;
                    pause(0.01);
                end
                if waitingflag == 0
                    return;
                end
            end
            
        end
        
        
        %% function to check and report errors
        % input: No
        % output: diplay coresponding errors to the command window if any
        function CheckError(self)
            
            % get the result
            CommStatus = int32(calllib('dynamixel','dxl_get_result'));
            % delay to receive the result
            pause(0.02);
            
            if CommStatus == self.COMM_RXSUCCESS
                % errors from dynamixel
                if int32(calllib('dynamixel','dxl_get_rxpacket_error', self.ERRBIT_VOLTAGE))==1
                    disp('Input Voltage Error!');
                elseif int32(calllib('dynamixel','dxl_get_rxpacket_error',self.ERRBIT_ANGLE))==1
                    disp('Angle limit error!');
                elseif int32(calllib('dynamixel','dxl_get_rxpacket_error',self.ERRBIT_OVERHEAT))==1
                    disp('Overheat error!');
                elseif int32(calllib('dynamixel','dxl_get_rxpacket_error',self.ERRBIT_RANGE))==1
                    disp('Out of range error!');
                elseif int32(calllib('dynamixel','dxl_get_rxpacket_error',self.ERRBIT_CHECKSUM))==1
                    disp('Checksum error!');
                elseif int32(calllib('dynamixel','dxl_get_rxpacket_error',self.ERRBIT_OVERLOAD))==1
                    disp('Overload error!');
                elseif int32(calllib('dynamixel','dxl_get_rxpacket_error',self.ERRBIT_INSTRUCTION))==1
                    disp('Instruction code error!');
                end
            else
                % errors relative to communication
                switch(CommStatus)
                    case self.COMM_TXFAIL
                        disp('COMM_TXFAIL : Failed transmit instruction packet!');
                    case self.COMM_TXERROR
                        disp('COMM_TXERROR: Incorrect instruction packet!');
                    case self.COMM_RXFAIL
                        disp('COMM_RXFAIL: Failed get status packet from device!');
                    case self.COMM_RXWAITING
                        disp('COMM_RXWAITING: Now recieving status packet!');
                    case self.COMM_RXTIMEOUT
                        disp('COMM_RXTIMEOUT: There is no status packet!');
                    case self.COMM_RXCORRUPT
                        disp('COMM_RXCORRUPT: Incorrect status packet!');
                    otherwise
                        disp('This is unknown error code!');
                end
            end
            
        end
        
        
        %% function to map the rotating angle to goal position
        % mapping 0бу~300бу to 0 ~ 1023
        % input:
        % ouput:
        function position = Angle2Position(self,angle)
            
            position = int32(angle*self.ANGLE_RESOLUTION/self.ANGLE_RANGE + self.AmpPos);
            
        end
        
        
        %% function to map the goal position to the rotating angle
        % mapping 0~1023 to 0бу~300бу
        % input:
        % output:
        function angle = Position2Angle(self,position)
            
            angle = (position - self.AmpPos)*self.ANGLE_RANGE/self.ANGLE_RESOLUTION;
            
        end
        
        
        %% function to convert speed from rpm (maximum 59) to binany value
        % input:
        % output:
        function speed = RPM2BinSpeed(self,rpm)
            
            speed = ones(1,4) * rpm * 1023 / self.MaxSpeed;
            
        end
        
    end
    
    
end


%% rad 2 degree
function deg = rad2deg(rad)

deg = rad*180/pi;

end