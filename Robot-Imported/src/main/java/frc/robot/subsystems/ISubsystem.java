package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public interface ISubsystem {
    /**
     * Method exists so subsystems can exist as an interface. Likley will return itself
     */
    public SubsystemBase asSubsystem();
}