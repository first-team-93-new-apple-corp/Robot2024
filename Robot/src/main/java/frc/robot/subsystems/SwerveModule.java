package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.print.CancelablePrintJob;

public class SwerveModule extends SubsystemBase {
  double Motor_Commands;
  double Turning_Degrees;

  AHRS Gyro;

  WPI_TalonFX Driving_Motor;
  WPI_TalonFX Turning_Motor;
  WPI_CANCoder Can_Coder;
  AbsoluteSensorRange Range;
  SimpleMotorFeedforward feedforward;

  PIDController TurningPID = new PIDController(
    DriveConstants.Turning_P,
    DriveConstants.Turning_I,
    DriveConstants.Turning_D
  );

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {
    feedforward = new SimpleMotorFeedforward(0.25,0,0);
    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Driving_Motor.setNeutralMode(NeutralMode.Brake);
    Driving_Motor.setInverted(true);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.supplyCurrLimit.enable = true;
    driveConfig.supplyCurrLimit.currentLimit = 5;
    driveConfig.supplyCurrLimit.triggerThresholdCurrent = 5;
    driveConfig.supplyCurrLimit.triggerThresholdTime = .254;
    Driving_Motor.configAllSettings(driveConfig);
    Turning_Motor = new WPI_TalonFX(turnMotorID);
    Turning_Motor.setNeutralMode(NeutralMode.Brake);

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);
    Range = AbsoluteSensorRange.valueOf(0);
    Can_Coder.configAbsoluteSensorRange(Range);

    TurningPID.setTolerance(DriveConstants.Turning_Tolerance);
    TurningPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // if 0,0,0 is in here after auto what happens?

    // optimize which way to turn the wheel
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState,
      getAngle()
    );

    //do not need PID on drive motors - just a simple voltage calculation
    double driveOutput =
      (state.speedMetersPerSecond / DriveConstants.Max_Strafe_Speed) *
      DriveConstants.Max_Volts;

    //Turning needs a pid because it has a setpoint it need to reach
    double turnOutput = TurningPID.calculate(
      getAngle().getRadians(),
      state.angle.getRadians()
      //why doesn't optimize or this fix this if states aren't recorded
    );
    double feedOutput = feedforward.calculate(
      Math.toRadians(Can_Coder.getVelocity())
    );

    Driving_Motor.setVoltage(driveOutput);
    Turning_Motor.setVoltage(turnOutput + feedOutput);
  }

  public void resetEncoders() { //need to figure out offsets
    Driving_Motor.setSelectedSensorPosition(0);
    Turning_Motor.setSelectedSensorPosition(0);
  }

  // calculate the current velocity of the driving wheel
  public double getVelocity() {
    double speed =
      Driving_Motor.getSelectedSensorVelocity() *
      10 /
      DriveConstants.TalonFX_Encoder_Resolution /
      DriveConstants.Driving_Gearing *
      DriveConstants.Wheel_Circumference;
    return Units.feetToMeters(speed);
  }

  // get angle from can coder
  //test to see if get absolute position is continuous
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Can_Coder.getAbsolutePosition());
  }

  @Override
  public void periodic() {
    // System.out.println(MathUtil.angleModulus(100000));
    //this for example does wrap the angle

  }

  @Override
  public void simulationPeriodic() {}
}
