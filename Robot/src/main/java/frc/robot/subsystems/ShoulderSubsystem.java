package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase implements GenericMotorSubsystem {

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
    Shoulder_FL = new WPI_TalonFX(Constants.CanID_CTRE.FrontLeftShoulder);
    Shoulder_BL = new WPI_TalonFX(Constants.CanID_CTRE.BackLeftShoulder);

    Shoulder_FR = new WPI_TalonFX(Constants.CanID_CTRE.FrontRightShoulder);
    Shoulder_BR = new WPI_TalonFX(Constants.CanID_CTRE.BackRightShoulder);

    Shoulder_FR.setInverted(TalonFXInvertType.Clockwise);
    Shoulder_BR.setInverted(TalonFXInvertType.Clockwise);

    Shoulder_FL.setInverted(TalonFXInvertType.CounterClockwise);
    Shoulder_BL.setInverted(TalonFXInvertType.CounterClockwise);

    // Use Motor Controller group once all motors spin the same direction
    ShoulderMotors = new MotorControllerGroup(Shoulder_FL, Shoulder_FR, Shoulder_BL, Shoulder_BR);


    ShoulderMotorConfig = new TalonFXConfiguration();

    shoulderCanCoder = new CANCoder(Constants.CanID_CTRE.ShoulderCancoder); // verify ids
    shoulderCanCoder.configSensorDirection(true);
    shoulderCanCoder.configMagnetOffset(-219.462890625);

    kP = 0.7;
    kI = 0;
    kD = 0;

    MAXVELO = 200;
    MAXACCEL = 200;

    double currentAngle = getAbsoluteDegrees();
    currentAngle = DegreesToTicks(currentAngle);

    Shoulder_FL.setSelectedSensorPosition(currentAngle);
    Shoulder_FR.setSelectedSensorPosition(currentAngle);
    Shoulder_BL.setSelectedSensorPosition(currentAngle);
    Shoulder_BR.setSelectedSensorPosition(currentAngle);

    ShoulderMotorConfig.slot0.kP = kP;
    ShoulderMotorConfig.slot0.kI = kI;
    ShoulderMotorConfig.slot0.kD = kD;
    ShoulderMotorConfig.motionAcceleration = MAXACCEL;
    ShoulderMotorConfig.motionCurveStrength = 4;
    ShoulderMotorConfig.motionCruiseVelocity = MAXVELO; // max 1600
    ShoulderMotorConfig.voltageCompSaturation = 12; // check
    ShoulderMotorConfig.slot0.closedLoopPeakOutput = 0.3;

    Shoulder_FL.enableVoltageCompensation(true);
    Shoulder_FR.enableVoltageCompensation(true);
    Shoulder_BL.enableVoltageCompensation(true);
    Shoulder_BR.enableVoltageCompensation(true);

    Shoulder_FL.configAllSettings(ShoulderMotorConfig);
    Shoulder_FR.configAllSettings(ShoulderMotorConfig);
    Shoulder_BL.configAllSettings(ShoulderMotorConfig);
    Shoulder_BR.configAllSettings(ShoulderMotorConfig);



    SmartDashboard.putNumber("Shoulder kI", kI);
    SmartDashboard.putNumber("Shoulder kP", kP);
    SmartDashboard.putNumber("Shoulder kD", kD);
    SmartDashboard.putNumber("Shoulder Velocity", MAXVELO);
    SmartDashboard.putNumber("Shoulder Acceleration", MAXACCEL);
    SmartDashboard.putNumber("Shoulder Max Output", 0);
    
    
  }

  public void toSetpoint(double angleInDegrees) { // TODO parameter should specify units.

    double angleInTicks = DegreesToTicks(angleInDegrees);

    Shoulder_FL.set(ControlMode.MotionMagic, angleInTicks);
    Shoulder_FR.set(ControlMode.MotionMagic, angleInTicks);
    Shoulder_BL.set(ControlMode.MotionMagic, angleInTicks);
    Shoulder_BR.set(ControlMode.MotionMagic, angleInTicks);

  }

  public void directMotorCommand(double speed) {
    ShoulderMotors.set(speed);
  }

  public void stopMotors() {
    ShoulderMotors.set(0);
  }

  // degrees of the shoulder
  public double DegreesToTicks(double degrees) {
    return (degrees) * Constants.DegreesToTicksShoulder;
  }

  // returns degrees of the shoulder
  public double TicksToDegrees(double Ticks) {
    return Ticks / (Constants.DegreesToTicksShoulder);
  }

  // public double getDegrees() {
  // return TicksToDegrees(Shoulder_FL.getSelectedSensorPosition());
  // }

  public double getAbsoluteDegrees() {
    return shoulderCanCoder.getAbsolutePosition() / 2;
  }
  // public void AbsoluteZero() {
  // Shoulder_FL.setSelectedSensorPosition(
  // (shoulderCanCoder.getAbsolutePosition() / 360) * (2048 *
  // Constants.ShoulderGearRatio));
  // }

  @Override
  public void periodic() {

    ShoulderMotorConfig.slot0.kP = SmartDashboard.getNumber("Shoulder kP", kP);
    ShoulderMotorConfig.slot0.kI = SmartDashboard.getNumber("Shoulder kI", kI);
    ShoulderMotorConfig.slot0.kD = SmartDashboard.getNumber("Shoulder kD", kD);
    ShoulderMotorConfig.motionCruiseVelocity = SmartDashboard.getNumber("Shoulder Velocity", MAXVELO);
    ShoulderMotorConfig.motionAcceleration  = SmartDashboard.getNumber("Shoulder Acceleration", MAXACCEL);
    ShoulderMotorConfig.slot0.closedLoopPeakOutput = SmartDashboard.getNumber("Shoulder Max Output", 0); 

    ShoulderMotorConfig.motionCurveStrength = 4;
    ShoulderMotorConfig.voltageCompSaturation = 12; // check

    Shoulder_FL.configAllSettings(ShoulderMotorConfig);
    Shoulder_FR.configAllSettings(ShoulderMotorConfig);
    Shoulder_BL.configAllSettings(ShoulderMotorConfig);
    Shoulder_BR.configAllSettings(ShoulderMotorConfig);

    SmartDashboard.putNumber("Shoulder Angle Degrees", shoulderCanCoder.getAbsolutePosition() / 2);

    SmartDashboard.putNumber("Sus Shoulder Angle in Ticks", Shoulder_FL.getSelectedSensorPosition());

    SmartDashboard.putNumber("Sus Shoulder Angle in Degrees", TicksToDegrees(Shoulder_FL.getSelectedSensorPosition()));

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
