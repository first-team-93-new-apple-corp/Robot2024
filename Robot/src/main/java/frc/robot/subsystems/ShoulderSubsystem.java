package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase implements GenericMotorSubsystem{


  WPI_TalonFX Shoulder_FL, Shoulder_FR, Shoulder_BL, Shoulder_BR;


  TalonFXConfiguration ShoulderMotorConfig;
  MotorControllerGroup ShoulderMotors;
  CANCoder shoulderCanCoder;
  DigitalInput ExtendedLimitSwitch;
  DigitalInput ClosedLimitSwitch;

  public double Setpoint = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double MAXVELO, MAXACCEL;

  public ShoulderSubsystem() {
    Shoulder_FL = new WPI_TalonFX(Constants.CanID_CTRE.FrontLeftShoulder); //TODO verify ids
    Shoulder_FR = new WPI_TalonFX(Constants.CanID_CTRE.FrontRightShoulder);
    Shoulder_BL = new WPI_TalonFX(Constants.CanID_CTRE.BackLeftShoulder);
    Shoulder_BR = new WPI_TalonFX(Constants.CanID_CTRE.BackRightShoulder);
    //Use Motor Controller group once all motors spin the same direction
    ShoulderMotors = new MotorControllerGroup(Shoulder_FL, Shoulder_FR, Shoulder_BL, Shoulder_BR);
    ShoulderMotorConfig = new TalonFXConfiguration(); 


    shoulderCanCoder = new CANCoder(0); //verify ids
    kP = 0.7;
    kI = 0;
    kD = 0;

    MAXVELO = 200;
    MAXACCEL = 200;

    ShoulderMotorConfig.slot0.kP = kP;
    ShoulderMotorConfig.slot0.kI = kI;
    ShoulderMotorConfig.slot0.kD = kD;
    ShoulderMotorConfig.motionAcceleration = MAXACCEL;
    ShoulderMotorConfig.motionCurveStrength = 4;
    ShoulderMotorConfig.motionCruiseVelocity = MAXVELO; //max 1600
    ShoulderMotorConfig.voltageCompSaturation = 12; // check

    Shoulder_FL.enableVoltageCompensation(true);
    Shoulder_FR.enableVoltageCompensation(true);
    Shoulder_BL.enableVoltageCompensation(true);
    Shoulder_BR.enableVoltageCompensation(true);

    Shoulder_FL.configAllSettings(ShoulderMotorConfig);
    Shoulder_FR.configAllSettings(ShoulderMotorConfig);
    Shoulder_BL.configAllSettings(ShoulderMotorConfig);
    Shoulder_BR.configAllSettings(ShoulderMotorConfig);
    SmartDashboard.putNumber("MaxOutputShoulder", 0);

    // SmartDashboard.putNumber("Arm Setpoint", 0);
    SmartDashboard.putNumber("CurrentPoseShoulder", Shoulder_FL.getSelectedSensorPosition());
    SmartDashboard.putNumber("kPShoulder", kP);
    SmartDashboard.putNumber("kIShoulder", kI);
    SmartDashboard.putNumber("kDShoulder", kD);
    SmartDashboard.putNumber("VelocityShoulder", MAXVELO);
    SmartDashboard.putNumber("AccelerationShoulder", MAXACCEL);

    Shoulder_FR.follow(Shoulder_FL);
    Shoulder_BL.follow(Shoulder_FL);
    Shoulder_BR.follow(Shoulder_FL);


  }
  public void toSetpoint(double TicksetPoint){ //TODO parameter should specify units.
  Shoulder_FL.set(ControlMode.MotionMagic, TicksetPoint);

  }

  public void directMotorCommand(double speed){
    Shoulder_FL.set(speed);
  }

  public void stopMotors() {
    Shoulder_FL.set(0);
  }

  public double DegreesToRotations(double degrees){
    return degrees * Constants.DegreesToTicksShoulder;
  }

  public double TicksToDegrees(double Ticks){
    return Ticks * 1/Constants.DegreesToTicksShoulder;
  }

  public double getDegrees(){
    return TicksToDegrees(Shoulder_FL.getSelectedSensorPosition());
  }

  public void AbsoluteZero(){
    Shoulder_FL.setSelectedSensorPosition((shoulderCanCoder.getAbsolutePosition() / 360) * (2048 * Constants.ShoulderGearRatio));
  }
  @Override public void periodic() {
    kP = SmartDashboard.getNumber("kPShoulder", kP);
    kI = SmartDashboard.getNumber("kIShoulder", kI);
    kD = SmartDashboard.getNumber("kDShoulder", kD);
    MAXVELO = SmartDashboard.getNumber("VelocityShoulder", MAXVELO);
    MAXACCEL = SmartDashboard.getNumber("AccelerationShoulder", MAXACCEL);

  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public boolean atSetpoint() {
    // TODO Auto-generated method stub
    return false;
  }
}
