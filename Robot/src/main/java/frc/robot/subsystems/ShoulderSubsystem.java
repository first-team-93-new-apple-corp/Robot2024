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

    double currentAngle = shoulderCanCoder.getAbsolutePosition(); 

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
    // ShoulderMotorConfig.slot0.closedLoopPeakOutput = 0.3; 

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
    SmartDashboard.putNumber("kPShoulder", kP);
    SmartDashboard.putNumber("kIShoulder", kI);
    SmartDashboard.putNumber("kDShoulder", kD);
    SmartDashboard.putNumber("VelocityShoulder", MAXVELO);
    SmartDashboard.putNumber("AccelerationShoulder", MAXACCEL);

    // Shoulder_FR.follow(Shoulder_FL);
    // Shoulder_BL.follow(Shoulder_FL);
    // Shoulder_BR.follow(Shoulder_FL);

  }

  public void toSetpoint(double TicksetPoint) { // TODO parameter should specify units.
    Shoulder_FL.set(ControlMode.MotionMagic, TicksetPoint);

  }

  public void directMotorCommand(double speed) {
    ShoulderMotors.set(speed);
  }

  public void stopMotors() {
    ShoulderMotors.set(0);
  }

  public double DegreesToTicks(double degrees) {
    return (degrees / 360) * Constants.DegreesToTicksShoulder;
  }

  public double TicksToDegrees(double Ticks) {
    return Ticks * 1 / Constants.DegreesToTicksShoulder;
  }

  public double getDegrees() {
    return TicksToDegrees(Shoulder_FL.getSelectedSensorPosition());
  }

  public void AbsoluteZero() {
    Shoulder_FL.setSelectedSensorPosition(
        (shoulderCanCoder.getAbsolutePosition() / 360) * (2048 * Constants.ShoulderGearRatio));
  }

  @Override
  public void periodic() {
    kP = SmartDashboard.getNumber("kPShoulder", kP);
    kI = SmartDashboard.getNumber("kIShoulder", kI);
    kD = SmartDashboard.getNumber("kDShoulder", kD);
    MAXVELO = SmartDashboard.getNumber("VelocityShoulder", MAXVELO);
    MAXACCEL = SmartDashboard.getNumber("AccelerationShoulder", MAXACCEL);


    SmartDashboard.putNumber("Shoulder Angle", shoulderCanCoder.getAbsolutePosition());

    SmartDashboard.putNumber("Sus Shoulder Angle", Shoulder_FL.getSelectedSensorPosition()); 



    


    

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
