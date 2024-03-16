package frc.robot.subsystems.Intake.IO;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public interface IntakeIO {

  public static class IntakeIOInputs {
      public boolean NoteInIntake;
      public boolean CrrentlyIntaking;
    }

    public void updateValues(IntakeIOInputs inputs);

    public void resetIntakeState();

    public void stop();

    public void Intake();

    public void passthrough();
}
