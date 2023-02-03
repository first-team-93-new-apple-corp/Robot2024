package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingSubsystem extends SubsystemBase implements GenericMotorSubsystem {

  enum TelescopeState {
    DEFAULT_STATE,
    GROUND_LOAD,
    PLAYER_LOAD,
    LOW_HYBRID,
    MID_CONE,
    MID_CUBE,
    HIGH_CUBE,
    HIGH_CONE
  }

  public WPI_TalonSRX TelescopingMotor1;

  final int MinTicks = 181;
  final int MaxTicks = 13363;
  TalonSRXConfiguration TelescopeConfig;
  DigitalInput ExtendedLimitSwitch;
  DigitalInput ClosedLimitSwitch;

  public double Setpoint = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double MAXVELO, MAXACCEL;

  public TelescopingSubsystem() {
    TelescopingMotor1 = new WPI_TalonSRX(Constants.CanID_CTRE.TelescopingMotor);
    TelescopeConfig = new TalonSRXConfiguration();
    TelescopingMotor1.setNeutralMode(NeutralMode.Brake);
    kP = 1.2;
    kI = 0;
    kD = 0.5;

    MAXVELO = 2000;
    MAXACCEL = 4000;

    TelescopeConfig.slot0.kP = kP;
    TelescopeConfig.slot0.kI = kI;
    TelescopeConfig.slot0.kD = kD;
    TelescopeConfig.motionAcceleration = MAXACCEL;
    TelescopeConfig.motionCurveStrength = 4;
    TelescopeConfig.motionCruiseVelocity = MAXVELO; // max 1600

    ExtendedLimitSwitch = new DigitalInput(0);
    ClosedLimitSwitch = new DigitalInput(9);

    TelescopingMotor1.configAllSettings(TelescopeConfig);
    SmartDashboard.getNumber("MaxOutput", 0);
    TelescopingMotor1.setSelectedSensorPosition(0);

    TelescopingMotor1.setSensorPhase(true);

    // SmartDashboard.putNumber("Arm Setpoint", 0);
    SmartDashboard.putNumber("CurrentPose", TicksToInchesTelescope(TelescopingMotor1.getSelectedSensorPosition()));
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("Velocity", MAXVELO);
    SmartDashboard.putNumber("Acceleration", MAXACCEL);

  }

  enum test {
    Forward,
    Stop_Backwards,
    Stop_Forwards,
    Backwards
  }

  // TODO: Update this value so we don't hit limit switches

  double limit = 2000;

  public void toSetpoint(double Setpoint) {

    // if we arent extending and we are within 500 ticks of our limit switch, then
    // stop
    if ((Setpoint < getTicks()) && (getTicks() - limit) < MinTicks) {
      TelescopingMotor1.set(0);

    }
    // if we are extending and we are within 500 of our extended limit switch, then
    // stop
    else if ((Setpoint > getTicks()) && ((getTicks() + limit) > MaxTicks)) {

      TelescopingMotor1.set(0);

    }

    // if our zero limit switch is triggered and we aren't extending
    else if (ClosedSwitchTriggered() && (Setpoint < getTicks())) {
      TelescopingMotor1.set(0);

    }
    // if our extended limit switch is triggered and we aren't retracting
    else if (ExtendedSwitchTriggered() && (Setpoint > getTicks())) {
      TelescopingMotor1.set(0);

    }
    // if all of those are somehow false, then we can actually run the arm lmao...
    else {
      TelescopingMotor1.set(ControlMode.MotionMagic, Setpoint);

    }
  }

  public void stopMotors() {
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

  public void directMotorCommand(double speed) {
    TelescopingMotor1.set(speed);
  }

  // only returns true on rising edge

  boolean LastExtendedLimitSwitchState = false;
  boolean LastClosedLimitSwitchState = false;

  public boolean getRisingEdgeClosedSwitch() {

    // if we are triggered and weren't last control cycle, then we should return
    // true
    boolean currentStatus = ClosedSwitchTriggered();

    // if these are both true, then we return false
    boolean return_value = (currentStatus && !LastClosedLimitSwitchState);

    // setting
    LastClosedLimitSwitchState = currentStatus;

    if (return_value) {
      System.out.println("IT WORKS");
    }

    return return_value;

  }

  public boolean getRisingEdgeExtendedSwitch() {

    // if we are triggered and weren't last control cycle, then we should return
    // true
    boolean currentStatus = ExtendedSwitchTriggered();

    // if these are both true, then we return false
    boolean return_value = (currentStatus && !LastExtendedLimitSwitchState);

    // setting
    LastExtendedLimitSwitchState = currentStatus;

    if (return_value) {
      System.out.println("IT WORKS");
    }

    return return_value;

  }

  public boolean ClosedSwitchTriggered() {
    return !ClosedLimitSwitch.get();
  }

  // only returns true on rising edge

  public boolean ExtendedSwitchTriggered() {
    return !ExtendedLimitSwitch.get();
  }

  public double InchesToTicksTelescope(double Inches) {
    return Inches * Constants.InchesToTicksTelescope;
  }

  public double TicksToInchesTelescope(double Ticks) {
    return Ticks / Constants.InchesToTicksTelescope;
  }

  public void SetEncoder() {
    if (getRisingEdgeClosedSwitch()) {
      TelescopingMotor1.setSelectedSensorPosition(MinTicks);
    } else if (getRisingEdgeExtendedSwitch()) {
      TelescopingMotor1.setSelectedSensorPosition(MaxTicks);

    }
  }

  public double getTicks() {
    return TelescopingMotor1.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Arm Setpoint", Setpoint);
    double position = TelescopingMotor1.getSelectedSensorPosition();

    SmartDashboard.putNumber("Arm Current Position", position);

    // SmartDashboard.putBoolean("Closed Triggered", getRisingEdgeClosedSwitch());
    // SmartDashboard.putBoolean("Extended Triggered",
    // getRisingEdgeExtendedSwitch());

    TelescopingMotor1.config_kP(0, SmartDashboard.getNumber("kP", kP));
    TelescopingMotor1.config_kD(0, SmartDashboard.getNumber("kD", kD));
    TelescopeConfig.motionCruiseVelocity = SmartDashboard.getNumber("Velocity", MAXVELO);
    TelescopeConfig.motionAcceleration = SmartDashboard.getNumber("Acceleration", MAXACCEL);

    // TelescopingMotor1.configClosedLoopPeakOutput(0,
    // SmartDashboard.getNumber("MaxOutput", 0));

    // TelescopeConfig.slot0.kD = SmartDashboard.getNumber("kD", 0);
    // TelescopingMotor1.setSensorPhase(true);

    SetEncoder();

  }

  @Override
  public void simulationPeriodic() {
  }

  // TODO: Test this and see if it works
  @Override
  public boolean atSetpoint() {
    return (Math.abs(TelescopingMotor1.getSelectedSensorPosition() - Setpoint) < 0.2);
  }
}
