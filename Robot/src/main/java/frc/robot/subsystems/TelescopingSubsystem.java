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
import edu.wpi.first.wpilibj.Timer;
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
  DigitalInput ExtendedLimitSwitch;
  DigitalInput ClosedLimitSwitch;

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

    ExtendedLimitSwitch = new DigitalInput(0);
    ClosedLimitSwitch = new DigitalInput(9);

    TelescopeConfig.slot0.closedLoopPeakOutput = 0.15;
    TelescopingMotor1.configAllSettings(TelescopeConfig);
    SmartDashboard.putNumber("Setpoint", Setpoint);
    SmartDashboard.putNumber("CurrentPose", TicksToInchesTelescope(TelescopingMotor1.getSelectedSensorPosition()));
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

  }

  enum test {
    Forward,
    Stop_Backwards,
    Stop_Forwards,
    Backwards

  }

  public void stopMotor(){
    TelescopingMotor1.set(0); 
  }
  test current_state = test.Backwards;
  Timer testing = new Timer();
  double speed = 0.2; 

  public void OscilateArm() {

    switch (current_state) {
      case Forward:
        if (!ExtendedLimitSwitch.get()) {
          current_state = test.Stop_Backwards;
          TelescopingMotor1.set(0);
          testing.reset();
          testing.start();

        } else {
          TelescopingMotor1.set(speed);

        }
        break;
      case Backwards:
        if (!ClosedLimitSwitch.get()) {
          TelescopingMotor1.setSelectedSensorPosition(0); 

          current_state = test.Stop_Forwards;
          TelescopingMotor1.set(0);
          testing.reset();
          testing.start();
        } else {
          TelescopingMotor1.set(-speed);

        }
        break;

      case Stop_Backwards:
        if (testing.advanceIfElapsed(2)) {
          testing.stop();
          current_state = test.Backwards;
        }
        break;
      case Stop_Forwards:
        if (testing.advanceIfElapsed(2)) {
          testing.stop();
          current_state = test.Forward;
        }
        break;
      default:
        break;

    }

    System.out.println(TelescopingMotor1.getSelectedSensorPosition());
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

  // public void zeroEncoder() {
  //   if (ClosedLimitSwitch.get()) {
  //     TelescopingMotor1.setSelectedSensorPosition(0);
  //   } else if (ExtendedLimitSwitch.get()) {
  //     // Set Extended Ticks Here
  //     // TelescopingMotor1.setSelectedSensorPosition()

  //   }
  // }

  @Override
  public void periodic() {
    // zeroEncoder();
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
