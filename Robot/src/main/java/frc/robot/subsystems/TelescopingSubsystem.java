package frc.robot.subsystems;

import javax.naming.LimitExceededException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    TelescopingMotor1.setNeutralMode(NeutralMode.Brake);
    kP = 0.7;
    kI = 0;
    kD = 0.5;

    TelescopeConfig.slot0.kP = kP;
    TelescopeConfig.slot0.kI = kI;
    TelescopeConfig.slot0.kD = kD;

    ExtendedLimitSwitch = new DigitalInput(0);
    ClosedLimitSwitch = new DigitalInput(9);

    TelescopeConfig.slot0.closedLoopPeakOutput = 1;

    TelescopingMotor1.configAllSettings(TelescopeConfig);
    TelescopingMotor1.setSensorPhase(true);
    SmartDashboard.putNumber("Setpoint", Setpoint);
    SmartDashboard.putNumber("CurrentPose", TicksToInchesTelescope(TelescopingMotor1.getSelectedSensorPosition()));
    SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

  }

  enum test {
    Forward,
    Stop_Backwards,
    Stop_Forwards,
    Backwards
  }


  public void toSetpoint(int Setpoint){

    TelescopingMotor1.set(ControlMode.Position, Setpoint);
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
  public void directMotorCommand(double speed){
    TelescopingMotor1.set(speed);
  }

  public double InchesToTicksTelescope(double Inches) {
    return Inches * Constants.InchesToTicksTelescope;
  }

  public double TicksToInchesTelescope(double Ticks) {
    return Ticks / Constants.InchesToTicksTelescope;
  }

  // TODO: Set value only on rising edge and set closed to 406 and extend to 13172
  public void SetEncoder() {
    if (!ClosedLimitSwitch.get()) {
      TelescopingMotor1.setSelectedSensorPosition(0);
    } else if (!ExtendedLimitSwitch.get()) {
      // Set Extended Ticks Here
      TelescopingMotor1.setSelectedSensorPosition(13537); 

    }
  }

  @Override
  public void periodic() {
    // zeroEncoder();
    SmartDashboard.putNumber("Setpoint", Setpoint);
    double position = TelescopingMotor1.getSelectedSensorPosition(); 

    // if(position < 0){
    //   position *= -1; 
    // }
    SmartDashboard.putNumber("CurrentPose", position);
    TelescopingMotor1.config_kP(0, SmartDashboard.getNumber("kP", kP));
    TelescopingMotor1.config_kD(0, SmartDashboard.getNumber("kD", kD));

    // TelescopeConfig.slot0.kD = SmartDashboard.getNumber("kD", 0);
    // TelescopingMotor1.configAllSettings(TelescopeConfig);
    // TelescopingMotor1.setSensorPhase(true);
    
    SetEncoder();

  }


  @Override
  public void simulationPeriodic() {
  }
}
