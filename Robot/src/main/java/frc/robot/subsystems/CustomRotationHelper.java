package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//7777777777777777
public class CustomRotationHelper extends SubsystemBase {
    POVButton pov0;
    POVButton pov45;
    POVButton pov90;
    POVButton pov135;
    POVButton pov180;
    POVButton pov225;
    POVButton pov270;
    POVButton pov315;

    Joystick m_Joystick; 

    public enum Direction {
        NONE,
        FORWARDS,
        BACKWARDS
    }

    private Direction CurrentDirection;

    public CustomRotationHelper(Joystick joystick) {
        m_Joystick = joystick; 
        
        pov0 = new POVButton(joystick, 0); // front
        pov45 = new POVButton(joystick, 45); // fr wheel
        pov90 = new POVButton(joystick, 90); // right
        pov135 = new POVButton(joystick, 135); // bl wheel
        pov180 = new POVButton(joystick, 180); // back
        pov225 = new POVButton(joystick, 225);// bl wheel
        pov270 = new POVButton(joystick, 270);// left
        pov315 = new POVButton(joystick, 315); // fl wheel

        CurrentDirection = Direction.NONE;
    }

    public Direction getDirection() {

        return CurrentDirection;
    }

    public Translation2d povButton(){
    if(pov0.getAsBoolean()){
    return DriveConstants.Front;
    }
    else if(pov45.getAsBoolean()){
    return DriveConstants.Location_FR;
    }
    else if(pov90.getAsBoolean()){
    return DriveConstants.Right;
    }
    else if(pov135.getAsBoolean()){
    return DriveConstants.Location_BR;
    }
    else if(pov180.getAsBoolean()){
    return DriveConstants.Back;
    }
    else if(pov225.getAsBoolean()){
    return DriveConstants.Location_BL;
    }
    else if(pov270.getAsBoolean()){
    return DriveConstants.Left;
    }
    else if(pov315.getAsBoolean()){
    return DriveConstants.Location_FL;
    }
    else{
    return DriveConstants.Center;
    }
    }

    private void updateDirection() {
        
        int POV_Angle = m_Joystick.getPOV(); 
        if(POV_Angle == 0 ||POV_Angle == 45 || POV_Angle == 315){
            CurrentDirection = Direction.FORWARDS;

        }
        else if(POV_Angle == 180 || POV_Angle == 135 || POV_Angle == 225){
            CurrentDirection = Direction.BACKWARDS;
        }
        else if (POV_Angle == 90 || POV_Angle == 270){
            CurrentDirection = Direction.NONE;
        }
        
    }

    @Override
    public void periodic() {
        updateDirection();

    }

    @Override
    public void simulationPeriodic() {
    }

}
