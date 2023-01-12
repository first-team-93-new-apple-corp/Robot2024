package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingSubsystem extends SubsystemBase {

  enum TelescopeState {
    DEFAULT_STATE,
    GROUND_LOAD,
    PLAYER_LOAD,
    MID_CONE,
    MID_CUBE,
    HIGH_CUBE,
    HIGH_CONE,
    LOW_HYBRID
  }

  public CANSparkMax TelescopingMotor1; 
  RelativeEncoder TelescopingMotor1Encoder;

  SparkMaxPIDController TelescopingPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public TelescopingSubsystem() {
  TelescopingMotor1 = new CANSparkMax(0, MotorType.kBrushless);
  TelescopingMotor1Encoder= TelescopingMotor1.getEncoder();
    TelescopingPIDController = TelescopingMotor1.getPIDController();
  kP = 0.1; 
  kI = 1e-4;
  kD = 1; 
  kIz = 0; 
  kFF = 0; 
  kMaxOutput = 1; 
  kMinOutput = -1;

  // set PID coefficients
  TelescopingPIDController.setP(kP);
  TelescopingPIDController.setI(kI);
  TelescopingPIDController.setD(kD);
  TelescopingPIDController.setIZone(kIz);
  TelescopingPIDController.setFF(kFF);
  TelescopingPIDController.setOutputRange(kMinOutput, kMaxOutput);

  }
  public void RunTelescopingMotors(double SetPoint){
  TelescopingPIDController.setReference(InchestoRotations(SetPoint), CANSparkMax.ControlType.kPosition);
  }
  public double InchestoRotations(double Inches){
    return Inches * Constants.InchesToRotationsTelescope;
  }
 

  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
