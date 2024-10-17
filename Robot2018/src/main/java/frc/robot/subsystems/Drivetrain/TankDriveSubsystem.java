package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TankDriveSubsystem implements Subsystem {
    private final VictorSPX frontLeft;
    private final VictorSPX frontRight;
    private final VictorSPX backLeft;
    private final VictorSPX backRight;

    private final DifferentialDrive m_drivetrain;

    public TankDriveSubsystem() {
        frontLeft = new VictorSPX(5);
        frontRight = new VictorSPX(4);
        backLeft = new VictorSPX(0);
        backRight = new VictorSPX(10);
        
        //TODO: This needs to be changed if the motors fight each other
        backLeft.setInverted(false);
        backRight.setInverted(false);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        m_drivetrain = new DifferentialDrive(
                (speed) -> frontLeft.set(VictorSPXControlMode.PercentOutput, speed),
                (speed) -> frontRight.set(VictorSPXControlMode.PercentOutput, speed));

    }

    public void drive(double xSpeed, double zRotation) {
        m_drivetrain.arcadeDrive(xSpeed, zRotation);
    }

}
