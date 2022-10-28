package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  double Motor_Commands;
  double Turning_Degrees;

  AHRS Gyro;

  WPI_TalonFX Driving_Motor;
  WPI_TalonFX Turning_Motor;
  WPI_CANCoder Can_Coder;
  AbsoluteSensorRange Range;

  PIDController TurningPID = new PIDController(DriveConstants.Turning_P, DriveConstants.Turning_I, DriveConstants.Turning_D);

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {

    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Driving_Motor.setNeutralMode(NeutralMode.Brake);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.supplyCurrLimit.enable = true;
    driveConfig.supplyCurrLimit.currentLimit = 80;
    driveConfig.supplyCurrLimit.triggerThresholdCurrent = 120;
    driveConfig.supplyCurrLimit.triggerThresholdTime = 2;
    Driving_Motor.configAllSettings(driveConfig);
    Turning_Motor = new WPI_TalonFX(turnMotorID);
    Turning_Motor.setNeutralMode(NeutralMode.Brake);

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);
    Range = AbsoluteSensorRange.valueOf(1);
    Can_Coder.configAbsoluteSensorRange(Range);

    TurningPID.setTolerance(DriveConstants.Turning_Tolerance);
    TurningPID.enableContinuousInput(-Math.PI, Math.PI);


  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(getVelocity(), calculateAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState) {

    // optimize which way to turn the wheel
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState,
      calculateAngle()
    );

    //do not need PID on drive motors - just a simple voltage calculation
    double driveOutput =
      (state.speedMetersPerSecond / DriveConstants.Max_Strafe_Speed) *
      DriveConstants.Max_Volts;

    //Turning needs a pid because it has a setpoint it need to reach
    double turnOutput = TurningPID.calculate(
      calculateAngle().getRadians(),
      state.angle.getRadians()
    );

    Driving_Motor.setVoltage(driveOutput);
    Turning_Motor.setVoltage(turnOutput);
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
  public Rotation2d calculateAngle() {
    return Rotation2d.fromDegrees(Can_Coder.getAbsolutePosition());
  }



  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
