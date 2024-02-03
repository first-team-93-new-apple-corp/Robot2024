package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX frontIntake;
    private TalonFX backIntake;

    private double IntakeSpeed = 0.75;
    private double PassoverSpeed = 0.5;

    public IntakeSubsystem() {
        frontIntake = new TalonFX(Constants.CTRE.Canivore.F_Intake, "drivetrain");
        backIntake = new TalonFX(Constants.CTRE.Canivore.B_Intake, "drivetrain");
        backIntake.setInverted(false);
    }

    public void Intake() {
        frontIntake.set(IntakeSpeed);
        backIntake.set(IntakeSpeed);
    }

    public void passthrough() {
        frontIntake.set(PassoverSpeed);
        backIntake.set(PassoverSpeed);
    }

    public void stop() {
        frontIntake.set(0);
        backIntake.set(0);
    }
}
