package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve.TunerConstants;

import static edu.wpi.first.units.Units.*;

public interface ControllerIO {
    //WIP
    //FUTURE REWORK OF THE CONTROLLER IO I ALREADY MADE
    // -SAWYER
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    public double InputLeft();
    public double InputUp();
    public double InputTheta();
    public Translation2d POV();
    public Trigger Seed();

    public default double DriveLeft(){
        return InputLeft() *  MaxSpeed;
    }
    
    public default double DriveUp(){
        return InputUp() *  MaxSpeed;
    }
    
    public default double DriveTheta(){
        return InputTheta() *  MaxAngularRate;
    }
}
