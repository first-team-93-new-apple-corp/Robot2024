package frc.robot.subsystems;

import javax.naming.LimitExceededException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public WPI_TalonSRX TelescopingMotor1;
  TalonSRXConfiguration TelescopeConfig;
  DigitalInput limitSwitch;

  public double Setpoint = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public TelescopingSubsystem() {
    TelescopingMotor1 = new WPI_TalonSRX(16);
    TelescopeConfig = new TalonSRXConfiguration();
    kP = 0.025;
    kI = 0;
    kD = 0;

    TelescopeConfig.slot0.kP = kP;
    TelescopeConfig.slot0.kI = kI;
    TelescopeConfig.slot0.kD = kD;
    limitSwitch = new DigitalInput(0);
    TelescopeConfig.slot0.closedLoopPeakOutput = 0.15;
    TelescopingMotor1.configAllSettings(TelescopeConfig);
    SmartDashboard.putNumber("Setpoint", Setpoint);
    SmartDashboard.putNumber("CurrentPose", TicksToInchesTelescope(TelescopingMotor1.getSelectedSensorPosition()));
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

  }

  public void RunTelescopingMotors(double SetPoint) {
    this.Setpoint = SetPoint;
    TelescopingMotor1.set(ControlMode.Position, InchesToTicksTelescope(SetPoint));
  }

  public double InchesToTicksTelescope(double Inches) {
    return Inches * Constants.InchesToTicksTelescope;
  }

  public double TicksToInchesTelescope(double Ticks) {
    return Ticks / Constants.InchesToTicksTelescope;
  }
  public void zeroEncoder() {
    if(limitSwitch.get()){
      TelescopingMotor1.setSelectedSensorPosition(0);
    }
  }

  @Override
  public void periodic() {
    zeroEncoder();
    SmartDashboard.putNumber("Setpoint", Setpoint);
    SmartDashboard.putNumber("CurrentPose", TelescopingMotor1.getSelectedSensorPosition());
    TelescopeConfig.slot0.kP = SmartDashboard.getNumber("kP", 0);
    TelescopeConfig.slot0.kP = SmartDashboard.getNumber("kI", 0);
    TelescopeConfig.slot0.kD = SmartDashboard.getNumber("kI", 0);
    TelescopingMotor1.configAllSettings(TelescopeConfig);

  }

  @Override
  public void simulationPeriodic() {
  }
}
