package frc.robot.subsystems.Intake.IO;

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
