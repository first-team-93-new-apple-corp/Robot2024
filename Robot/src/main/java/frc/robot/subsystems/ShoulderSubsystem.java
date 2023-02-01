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

public class ShoulderSubsystem extends SubsystemBase implements ArmInterface{


  WPI_TalonFX ShoulderMotorMain;
  WPI_TalonFX ShoulderMotor2;
  WPI_TalonFX ShoulderMotor3;
  WPI_TalonFX ShoulderMotor4;
  TalonFXConfiguration ShoulderMotorConfig;
  MotorControllerGroup ShoulderMotors;
  CANCoder shoulderCanCoder;
  DigitalInput ExtendedLimitSwitch;
  DigitalInput ClosedLimitSwitch;
  public double Setpoint = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double MAXVELO, MAXACCEL;

  public ShoulderSubsystem() {
    ShoulderMotorMain = new WPI_TalonFX(0); //TODO verify ids
    ShoulderMotor2 = new WPI_TalonFX(0);
    ShoulderMotor3 = new WPI_TalonFX(0);
    ShoulderMotor4 = new WPI_TalonFX(0);
    //Use Motor Controller group once all motors spin the same direction
    ShoulderMotors = new MotorControllerGroup(ShoulderMotorMain, ShoulderMotor2,ShoulderMotor3, ShoulderMotor4);
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

    ShoulderMotorMain.enableVoltageCompensation(true);
    ShoulderMotor2.enableVoltageCompensation(true);
    ShoulderMotor3.enableVoltageCompensation(true);
    ShoulderMotor4.enableVoltageCompensation(true);

    ShoulderMotorMain.configAllSettings(ShoulderMotorConfig);
    ShoulderMotor2.configAllSettings(ShoulderMotorConfig);
    ShoulderMotor3.configAllSettings(ShoulderMotorConfig);
    ShoulderMotor4.configAllSettings(ShoulderMotorConfig);
    SmartDashboard.putNumber("MaxOutputShoulder", 0);

    // SmartDashboard.putNumber("Arm Setpoint", 0);
    SmartDashboard.putNumber("CurrentPoseShoulder", ShoulderMotorMain.getSelectedSensorPosition());
    SmartDashboard.putNumber("kPShoulder", kP);
    SmartDashboard.putNumber("kDShoulder", kD);
    SmartDashboard.putNumber("VelocityShoulder", MAXVELO);
    SmartDashboard.putNumber("AccelerationShoulder", MAXACCEL);
    ShoulderMotor2.follow(ShoulderMotorMain);
    ShoulderMotor3.follow(ShoulderMotorMain);
    ShoulderMotor4.follow(ShoulderMotorMain);


  }
  public void toSetpoint(double TicksetPoint){ //TODO parameter should specify units.
  ShoulderMotorMain.set(ControlMode.MotionMagic, TicksetPoint);

  }

  public void directMotorCommand(double speed){
    ShoulderMotorMain.set(speed);
  }

  public void stopMotors() {
    ShoulderMotorMain.set(0);
  }

  public double DegreesToRotations(double degrees){
    return degrees * Constants.DegreesToTicksShoulder;
  }

  public double TicksToDegrees(double Ticks){
    return Ticks * 1/Constants.DegreesToTicksShoulder;
  }

  public double getDegrees(){
    return TicksToDegrees(ShoulderMotorMain.getSelectedSensorPosition());
  }

  public void AbsoluteZero(){
  ShoulderMotorMain.setSelectedSensorPosition((shoulderCanCoder.getAbsolutePosition() / 360) * (2048 * Constants.ShoulderGearRatio));
  }
  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
