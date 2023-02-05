package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shoulder;

public class ShoulderSubsystem extends SubsystemBase implements GenericMotorSubsystem {

  WPI_TalonFX Shoulder_FL, Shoulder_FR, Shoulder_BL, Shoulder_BR;

  TalonFXConfiguration ShoulderMotorConfig;
  // MotorControllerGroup ShoulderMotors;
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

    Shoulder_BL.setNeutralMode(NeutralMode.Brake);
    Shoulder_BR.setNeutralMode(NeutralMode.Brake);
    Shoulder_FL.setNeutralMode(NeutralMode.Brake);
    Shoulder_FL.setNeutralMode(NeutralMode.Brake);

    

    Shoulder_FR.setInverted(TalonFXInvertType.Clockwise);
    Shoulder_BR.setInverted(TalonFXInvertType.Clockwise);

    Shoulder_FL.setInverted(TalonFXInvertType.CounterClockwise);
    Shoulder_BL.setInverted(TalonFXInvertType.CounterClockwise);

    // Use Motor Controller group once all motors spin the same direction
    // ShoulderMotors = new MotorControllerGroup(Shoulder_FL, Shoulder_FR, Shoulder_BL, Shoulder_BR);



    ShoulderMotorConfig = new TalonFXConfiguration();

    shoulderCanCoder = new CANCoder(Constants.CanID_CTRE.ShoulderCancoder); // verify ids
    shoulderCanCoder.configSensorDirection(true);
    shoulderCanCoder.configMagnetOffset(151.259765625 );

    kP = 0;
    kI = 0;
    kD = 0;

    MAXVELO = 0; //5457.92;
    MAXACCEL = 0; //1000;

    double currentAngle = getAbsoluteDegrees();
    currentAngle = DegreesToTicks(currentAngle);


    Shoulder_FL.setSelectedSensorPosition(currentAngle);
    Shoulder_FR.setSelectedSensorPosition(currentAngle);
    Shoulder_BL.setSelectedSensorPosition(currentAngle);
    Shoulder_BR.setSelectedSensorPosition(currentAngle);

    Shoulder_BL.selectProfileSlot(0, 0);

    ShoulderMotorConfig.slot0.kP = 0.000002;


    
    ShoulderMotorConfig.slot0.kI = 0;
    ShoulderMotorConfig.slot0.kD = 0;
    ShoulderMotorConfig.motionAcceleration = 10000;
    ShoulderMotorConfig.motionCurveStrength = 4;
    ShoulderMotorConfig.motionCruiseVelocity = 7500; // max 1600
    ShoulderMotorConfig.voltageCompSaturation = 12; // check
    ShoulderMotorConfig.slot0.closedLoopPeakOutput = 0.2;


    ShoulderMotorConfig.slot0.allowableClosedloopError = DegreesToTicks(0.2);


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
    // SmartDashboard.putNumber("Shoulder Ramp", 0);

    
    
  }

  public void toSetpoint(double angleInDegrees) { // TODO parameter should specify units.

    double angleInTicks = DegreesToTicks(angleInDegrees);

    // Shoulder_FL.set(ControlMode.MotionMagic, angleInTicks);
    // Shoulder_FR.set(ControlMode.Follower, Constants.CanID_CTRE.FrontLeftShoulder);
    // Shoulder_BL.set(ControlMode.Follower, Constants.CanID_CTRE.FrontLeftShoulder);
    // Shoulder_BR.set(ControlMode.Follower, Constants.CanID_CTRE.FrontLeftShoulder);

    
    Shoulder_FR.set(ControlMode.MotionMagic, angleInTicks);
    Shoulder_FL.set(ControlMode.Follower, Constants.CanID_CTRE.FrontRightShoulder);
    Shoulder_BL.set(ControlMode.Follower, Constants.CanID_CTRE.FrontRightShoulder);
    Shoulder_BR.set(ControlMode.Follower, Constants.CanID_CTRE.FrontRightShoulder);


  


  }

  public void directMotorCommand(double speed) {
    // ShoulderMotors.set(speed);

    
    Shoulder_FR.set(ControlMode.PercentOutput, speed);
    Shoulder_FL.set(ControlMode.Follower, Constants.CanID_CTRE.FrontRightShoulder);
    Shoulder_BL.set(ControlMode.Follower, Constants.CanID_CTRE.FrontRightShoulder);
    Shoulder_BR.set(ControlMode.Follower, Constants.CanID_CTRE.FrontRightShoulder);
  }

  public void stopMotors() {
    // ShoulderMotors.set(0);

    Shoulder_FR.set(0);
    Shoulder_FL.set(0);
    Shoulder_BL.set(0);
    Shoulder_BR.set(0);
  }

  // degrees of the shoulder
  public double DegreesToTicks(double degrees) {
    return (((degrees / 360) * Constants.DegreesToTicksShoulder) );
  }

  // returns degrees of the shoulder
  public double TicksToDegrees(double Ticks) {
    return ((Ticks / (Constants.DegreesToTicksShoulder)) * 360);
  }

  // public double getDegrees() {
  // return TicksToDegrees(Shoulder_FL.getSelectedSensorPosition());
  // }

  public double getAbsoluteDegrees() {
    return shoulderCanCoder.getAbsolutePosition() /2;
  }
  // public void AbsoluteZero() {
  // Shoulder_FL.setSelectedSensorPosition(
  // (shoulderCanCoder.getAbsolutePosition() / 360) * (2048 *
  // Constants.ShoulderGearRatio));
  // }

  @Override
  public void periodic() {

    double Shoulder_kP = SmartDashboard.getNumber("Shoulder kP", 0);
    double Shoulder_kI = SmartDashboard.getNumber("Shoulder kP", 0);
    double Shoulder_kD = SmartDashboard.getNumber("Shoulder kP", 0);
    double Shoulder_MAXVELO = SmartDashboard.getNumber("Shoulder Velocity", 0);
    double Shoulder_MAXACCEL = SmartDashboard.getNumber("Shoulder Acceleration", 0);
    double Shoulder_MAXOUTPUT = SmartDashboard.getNumber("Shoulder Max Output", 0);
    double Shoulder_MAXRAMP = SmartDashboard.getNumber("Shoulder Ramp", 0);

    Shoulder_FL.config_kP(0, Shoulder_kP);
    Shoulder_FR.config_kP(0, Shoulder_kP);
    Shoulder_BL.config_kP(0, Shoulder_kP);
    Shoulder_BR.config_kP(0, Shoulder_kP);

    Shoulder_FL.config_kI(0, Shoulder_kI);
    Shoulder_FR.config_kI(0, Shoulder_kI);
    Shoulder_BL.config_kI(0, Shoulder_kI);
    Shoulder_BR.config_kI(0, Shoulder_kI);

    Shoulder_FL.config_kD(0, Shoulder_kD);
    Shoulder_FR.config_kD(0, Shoulder_kD);
    Shoulder_BL.config_kD(0, Shoulder_kD);
    Shoulder_BR.config_kD(0, Shoulder_kD);


    Shoulder_FL.configMotionCruiseVelocity(Shoulder_MAXVELO);
    Shoulder_FR.configMotionCruiseVelocity(Shoulder_MAXVELO);
    Shoulder_BL.configMotionCruiseVelocity(Shoulder_MAXVELO);
    Shoulder_BR.configMotionCruiseVelocity(Shoulder_MAXVELO);

    Shoulder_FL.configMotionAcceleration(Shoulder_MAXACCEL);
    Shoulder_FR.configMotionAcceleration(Shoulder_MAXACCEL);
    Shoulder_BL.configMotionAcceleration(Shoulder_MAXACCEL);
    Shoulder_BR.configMotionAcceleration(Shoulder_MAXACCEL); 

    Shoulder_FL.configClosedLoopPeakOutput(0, Shoulder_MAXOUTPUT);
    Shoulder_FR.configClosedLoopPeakOutput(0, Shoulder_MAXOUTPUT);
    Shoulder_BL.configClosedLoopPeakOutput(0, Shoulder_MAXOUTPUT);
    Shoulder_BR.configClosedLoopPeakOutput(0, Shoulder_MAXOUTPUT);
    
    // Shoulder_FL.configClosedloopRamp(Shoulder_MAXRAMP);
    // Shoulder_FR.configClosedloopRamp(Shoulder_MAXRAMP);
    // Shoulder_BL.configClosedloopRamp(Shoulder_MAXRAMP);
    // Shoulder_BR.configClosedloopRamp( Shoulder_MAXRAMP);
    



    // ShoulderMotorConfig.slot0.kI = SmartDashboard.getNumber("Shoulder kI", kI);
    // ShoulderMotorConfig.slot0.kD = SmartDashboard.getNumber("Shoulder kD", kD);
    // ShoulderMotorConfig.motionCruiseVelocity = SmartDashboard.getNumber("Shoulder Velocity", MAXVELO);
    // ShoulderMotorConfig.motionAcceleration  = SmartDashboard.getNumber("Shoulder Acceleration", MAXACCEL);
    // ShoulderMotorConfig.slot0.closedLoopPeakOutput = SmartDashboard.getNumber("Shoulder Max Output", 0); 


    // Shoulder_FL.configAllSettings(ShoulderMotorConfig);
    // Shoulder_FR.configAllSettings(ShoulderMotorConfig);
    // Shoulder_BL.configAllSettings(ShoulderMotorConfig);
    // Shoulder_BR.configAllSettings(ShoulderMotorConfig);

    SmartDashboard.putNumber("Shoulder Arm Degrees", getAbsoluteDegrees());

    SmartDashboard.putNumber("Shoulder Motor Angle in Ticks", Shoulder_FL.getSelectedSensorPosition());

    SmartDashboard.putNumber("Shoulder Motor Angle in Degrees", TicksToDegrees(Shoulder_FL.getSelectedSensorPosition()));


    SmartDashboard.putNumber("Shoulder Motor Speed", Shoulder_FR.getSelectedSensorVelocity(0));
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
