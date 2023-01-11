package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {



  enum ArmState {
    DEFAULT_STATE,
    GROUND_LOAD,
    PLAYER_LOAD,
    MID_CONE,
    MID_CUBE,
    HIGH_CUBE,
    HIGH_CONE,
    LOW_HYBRID
  }

  public CANSparkMax ShoulderMotor1; 
  public CANSparkMax ShoulderMotor2;
  RelativeEncoder ShoulderMotor1Encoder;
  RelativeEncoder ShoulderMotor2Encoder;

  SparkMaxPIDController ShoulderPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public ShoulderSubsystem() {
  ShoulderMotor1 = new CANSparkMax(0, MotorType.kBrushless);
  ShoulderMotor2 = new CANSparkMax(0, MotorType.kBrushless);
  ShoulderMotor1Encoder = ShoulderMotor1.getEncoder();
  ShoulderMotor2Encoder = ShoulderMotor2.getEncoder();
  ShoulderPIDController = ShoulderMotor1.getPIDController();
  ShoulderMotor1Encoder.getVelocity();

  kP = 0.1; 
  kI = 1e-4;
  kD = 1; 
  kIz = 0; 
  kFF = 0; 
  kMaxOutput = 1; 
  kMinOutput = -1;

  // set PID coefficients
  ShoulderPIDController.setP(kP);
  ShoulderPIDController.setI(kI);
  ShoulderPIDController.setD(kD);
  ShoulderPIDController.setIZone(kIz);
  ShoulderPIDController.setFF(kFF);
  ShoulderPIDController.setOutputRange(kMinOutput, kMaxOutput);

  }
  public void RunShoulderMotors(){
  double rotations = 0; //TODO
  ShoulderPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }

 

  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
