hArmController = ArmController(1:5,1);
hArmController.Pressbutton(1);
hArmController.Pressbutton(2);
hArmController.Pressbutton(3);
hArmController.Pressbutton(4);
hArmController.Pressbutton(5);
%pause(1)
hArmController.Pressbutton(6);
%pause(1);
hArmController.Pressbutton(7);
%pause(1);
hArmController.Pressbutton(8);
%pause(1);
hArmController.Pressbutton(9);

%hArmController.Terminate();
% delete(hGripperController_Left);