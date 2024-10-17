package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TankDriveSubsystem implements Subsystem {
    private VictorSPX frontLeft;
    private VictorSPX frontRight;
    private VictorSPX backLeft;
    private VictorSPX backRight;

    public TankDriveSubsystem() {
        frontLeft = new VictorSPX(0);
        frontRight = new VictorSPX(1);
        backLeft = new VictorSPX(2);
        backRight = new VictorSPX(3);

        // frontLeft.addFollower(backLeft);
    }
    
}
