package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveConstants;

public class CustomRotationHelper {
    POVButton pov0;
    POVButton pov45;
    POVButton pov90;
    POVButton pov135;
    POVButton pov180;
    POVButton pov225;
    POVButton pov270;
    POVButton pov315;

    public CustomRotationHelper(Joystick joystick) {
        pov0 = new POVButton(joystick, 0); //front
        pov45 = new POVButton(joystick, 45); // fr wheel
        pov90 = new POVButton(joystick, 90); //right
        pov135 = new POVButton(joystick, 135); //bl wheel
        pov180 = new POVButton(joystick, 180); //back
        pov225 = new POVButton(joystick, 225);// bl wheel
        pov270 = new POVButton(joystick, 270);//left
        pov315 = new POVButton(joystick, 315); //fl wheel
    }

    public Translation2d povButton(){
        if(pov0.get()){
            return DriveConstants.Front;
        }
        else if(pov45.get()){
            return DriveConstants.Location_FR;
        }
        else if(pov90.get()){
            return DriveConstants.Right;
        }
        else if(pov135.get()){
            return DriveConstants.Location_BR;
        }
        else if(pov180.get()){
            return DriveConstants.Back;
        }
        else if(pov225.get()){
            return DriveConstants.Location_BL;
        }
        else if(pov270.get()){
            return DriveConstants.Left;
        }
        else if(pov315.get()){
            return DriveConstants.Location_FL;
        }
        else{
            return DriveConstants.Center;
        }
}}
