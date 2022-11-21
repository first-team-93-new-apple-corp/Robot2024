package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  double Motor_Commands;
  double Turning_Degrees;

  AHRS Gyro;

  WPI_TalonFX Driving_Motor;
  WPI_TalonFX Turning_Motor;
  WPI_CANCoder Can_Coder;
  AbsoluteSensorRange Range;
  double LastAngle;
  double FinalAngle;
  SimpleMotorFeedforward feedForward;

  private Rotation2d previousAngle = new Rotation2d();

  double angleoffset;

  /* PID's are Unused for now. Leaving them in code. */
  PIDController TurningPID = new PIDController(
      DriveConstants.Turning_P,
      DriveConstants.Turning_I,
      DriveConstants.Turning_D);

  ProfiledPIDController TurningProfiledPID = new ProfiledPIDController(
      4.25,
      0,
      0.11,
      new Constraints(220, 600));

  public SwerveModule(
      int driveMotorID,
      int turnMotorID,
      int CanCoderID,
      double magnetOffset) {
    // feedForward = new SimpleMotorFeedforward(0.65, 0.216);
    feedForward = new SimpleMotorFeedforward(0.71, 0);
    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Driving_Motor.setNeutralMode(NeutralMode.Brake);
    Driving_Motor.setInverted(true);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    // driveConfig.supplyCurrLimit.enable = true;
    // driveConfig.supplyCurrLimit.currentLimit = 5;
    // driveConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    // driveConfig.supplyCurrLimit.triggerThresholdTime = .254;

    Driving_Motor.configFactoryDefault();
    Driving_Motor.configAllSettings(driveConfig);

    Turning_Motor = new WPI_TalonFX(turnMotorID);
    Turning_Motor.setNeutralMode(NeutralMode.Brake);

    /* This is Nolen test code no touchy */
    Turning_Motor.configFactoryDefault();
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    // turnConfig.supplyCurrLimit.enable = true;
    // turnConfig.supplyCurrLimit.currentLimit = 5;
    // turnConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    // turnConfig.supplyCurrLimit.triggerThresholdTime = .254;
    turnConfig.slot0.kP = DriveConstants.Turning_P;
    turnConfig.slot0.kI = DriveConstants.Turning_I;
    turnConfig.slot0.kD = DriveConstants.Turning_D;
    turnConfig.slot0.allowableClosedloopError = DriveConstants.Turning_Tolerance; 
    // turnConfig.slot0.

    Turning_Motor.configAllSettings(turnConfig);

    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);
    Range = AbsoluteSensorRange.valueOf(0);
    Can_Coder.configAbsoluteSensorRange(Range);
    Turning_Motor.setSelectedSensorPosition((Can_Coder.getAbsolutePosition() / 360.) * (2048 * 12.8));

    // TurningPID.setTolerance(DriveConstants.Turning_Tolerance);
    // TurningPID.enableContinuousInput(-Math.PI, Math.PI);
    // TurningProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
  }
    SwerveModuleState lastState = new SwerveModuleState(0, new Rotation2d());
  public void setDesiredState(SwerveModuleState desiredState) {


    double currentPos = Turning_Motor.getSelectedSensorPosition();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        Rotation2d.fromDegrees(Can_Coder.getAbsolutePosition()));

    double errorBound = (360) / 2.0;
    double m_positionError = MathUtil.inputModulus(state.angle.getDegrees() - (ticksToDegs(currentPos)), -errorBound,
        errorBound);
lastState = state;
    // do not need PID on drive motors - just a simple voltage calculation
    double driveOutput = (state.speedMetersPerSecond / DriveConstants.Max_Strafe_Speed) *
        DriveConstants.Max_Volts;

    Driving_Motor.setVoltage(driveOutput);

    Turning_Motor.set(ControlMode.Position, currentPos + degsToTicks(m_positionError));

  }

  public SwerveModuleState getState() {
    return lastState;
  }

  public double radsToTicks(double radians) {
    return radians / (2 * Math.PI / (12.8 * 2048));
  }

  public static double degsToTicks(double degrees) {
    return (degrees / 360) * (12.8 * 2048);
  }

  public static double  ticksToDegs(double ticks) {
    return ((ticks * 360) / (12.8 * 2048)) % 360;
  }

  // calculate the current velocity of the driving wheel
  public double getVelocity() { //sussy baka
    double speed = Driving_Motor.getSelectedSensorVelocity() *
        10 /
        DriveConstants.TalonFX_Encoder_Resolution /
        DriveConstants.Driving_Gearing *
        DriveConstants.Wheel_Circumference;
    return Units.feetToMeters(speed);
  }

  // get angle from can coder
  // test to see if get absolute position is continuous
  public Rotation2d getAngle() {
    return Rotation2d
        .fromDegrees((Turning_Motor.getSelectedSensorPosition() * 12.8) * 360 - (Can_Coder.getAbsolutePosition()));
  }

  @Override
  public void periodic() {
    // angleoffset = (Turning_Motor.getSelectedSensorPosition() * 12.8) * 360 -
    // (Can_Coder.getAbsolutePosition());
  }

  @Override
  public void simulationPeriodic() {
  }
}
