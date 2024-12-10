package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerIO {
    //WIP
    //FUTURE REWORK OF THE CONTROLLER IO I ALREADY MADE
    // -SAWYER
    public double DriveLeft();
    public double DriveUp();
    public Translation2d POV();
    public Trigger Seed();

    
}
