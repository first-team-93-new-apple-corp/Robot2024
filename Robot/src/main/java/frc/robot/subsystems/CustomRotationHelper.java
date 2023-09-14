package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
//7777777777777777
public class CustomRotationHelper extends SubsystemBase {
    

    Joystick m_Joystick; 

    public enum Direction {
        NONE,
        FORWARDS,
        BACKWARDS
    }

    private Direction CurrentDirection;

    public CustomRotationHelper(Joystick joystick) {
        m_Joystick = joystick; 
        
        POVButton pov0 = new POVButton(m_Joystick, 0); // front
        POVButton pov45 = new POVButton(m_Joystick, 45); // fr wheel
        POVButton pov90 = new POVButton(m_Joystick, 90); // right
        POVButton pov135 = new POVButton(m_Joystick, 135); // bl wheel
        POVButton pov180 = new POVButton(m_Joystick, 180); // back
        POVButton pov225 = new POVButton(m_Joystick, 225);// bl wheel
        POVButton pov270 = new POVButton(m_Joystick, 270);// left
        POVButton pov315 = new POVButton(m_Joystick, 315); // fl wheel
        
        if(pov0.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Front;
        } else if(pov45.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_FR;
        } else if(pov90.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Right;
        } else if(pov135.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_BR;
        } else if(pov180.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Back;
        } else if(pov225.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_BL;
        } else if(pov270.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Left;
        } else if(pov315.getAsBoolean()){
            DriveConstants.dCenter = DriveConstants.Location_FL;
        } else{
            DriveConstants.dCenter = new Translation2d(0,0);
        }


        CurrentDirection = Direction.NONE;

    }

    public Direction getDirection() {
        return CurrentDirection;
    }

    private void updateDirection() {
        
        int POV_Angle = m_Joystick.getPOV(); 
        if(POV_Angle == 0 || POV_Angle == 45 || POV_Angle == 315){
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
