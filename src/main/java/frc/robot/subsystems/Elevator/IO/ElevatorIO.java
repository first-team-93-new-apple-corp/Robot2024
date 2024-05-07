package frc.robot.subsystems.Elevator.IO;
import com.ctre.phoenix6.hardware.TalonFX;

public interface ElevatorIO {

  public static class ElevatorIOInputs {
      public boolean NoteInIntake;
      public boolean CrrentlyIntaking;
      public TalonFX ElevatorMotor;
    }

    public void updateValues(ElevatorIOInputs inputs);

    public void disable();

    public void initOnce();

    public boolean topLimitTriggered();

    public boolean bottomLimitTriggered();

    public double ElevatorPosition();

    public void runElevator();

    public void toSetpoint(double newSetpoint);

    public void zero();

    public boolean atSetpoint();
    public void stopElevator();

}
