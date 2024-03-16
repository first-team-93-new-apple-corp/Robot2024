package frc.robot.subsystems.Climber.IO;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;

public interface ClimberIO {

  public static class ClimberIOInputs {
    public TalonFX climberLeft;
    public TalonFX climberRight;
    public XboxController op;
    }

    public void updateValues(ClimberIOInputs inputs);

    public void leftSpeed(double speed);

    public void rightSpeed(double speed);
    
    public void zeroLeft();

    public void zeroRight();

    public double getLeftDraw();

    public double getRightDraw();

    public double leftPosition();

    public double rightPosition();

    public double checkLeftBound(double output);

    public double checkRightBound(double output);
}
