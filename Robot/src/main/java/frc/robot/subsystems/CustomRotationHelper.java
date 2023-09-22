package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
