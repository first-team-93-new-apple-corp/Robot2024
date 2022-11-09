package frc.robot.subsystems;

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
  SimpleMotorFeedforward feedforward;

  PIDController TurningPID = new PIDController(
    DriveConstants.Turning_P,
    DriveConstants.Turning_I,
    DriveConstants.Turning_D
  );
  ProfiledPIDController TurningProfiledPID = new ProfiledPIDController(
    DriveConstants.Turning_P,
    DriveConstants.Turning_I,
    DriveConstants.Turning_D,
    new Constraints(1, 6.28)
  );

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {
    feedforward = new SimpleMotorFeedforward(0.73644, 0.21334, 0.002268);
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

    /* Factory Default all hardware to prevent unexpected behaviour */
    Turning_Motor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    Turning_Motor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      Constants.kPIDLoopIdx,
      Constants.kTimeoutMs
    );

    /* Ensure sensor is positive when output is positive */
    Turning_Motor.setSensorPhase(Constants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase.
     */
    // Turning_Motor.setInverted(Constants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // Turning_Motor.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    Turning_Motor.configNominalOutputForward(0, Constants.kTimeoutMs);
    Turning_Motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    Turning_Motor.configPeakOutputForward(1, Constants.kTimeoutMs);
    Turning_Motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    Turning_Motor.configAllowableClosedloopError(
      0,
      Constants.kPIDLoopIdx,
      Constants.kTimeoutMs
    );

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    Turning_Motor.config_kF(
      Constants.kPIDLoopIdx,
      Constants.kGains.kF,
      Constants.kTimeoutMs
    );
    Turning_Motor.config_kP(
      Constants.kPIDLoopIdx,
      Constants.kGains.kP,
      Constants.kTimeoutMs
    );
    Turning_Motor.config_kI(
      Constants.kPIDLoopIdx,
      Constants.kGains.kI,
      Constants.kTimeoutMs
    );
    Turning_Motor.config_kD(
      Constants.kPIDLoopIdx,
      Constants.kGains.kD,
      Constants.kTimeoutMs
    );

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);
    Range = AbsoluteSensorRange.valueOf(0);
    Can_Coder.configAbsoluteSensorRange(Range);

    TurningPID.setTolerance(DriveConstants.Turning_Tolerance);
    TurningPID.enableContinuousInput(-Math.PI, Math.PI);
    TurningProfiledPID.enableContinuousInput(-Math.PI, Math.PI);
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
    double turnOutput = TurningProfiledPID.calculate(
      getAngle().getRadians(),
      state.angle.getRadians()
      //why doesn't optimize or this fix this if states aren't recorded
    );

    // double feedOutput = feedforward.calculate(turnOutput/12.0*6.28);

    Driving_Motor.setVoltage(driveOutput);

    Turning_Motor.set(TalonFXControlMode.Position, radsToTicks(state.angle.getRadians()));
  }

  public double radsToTicks(double radians){
    return radians * 2048; 
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
