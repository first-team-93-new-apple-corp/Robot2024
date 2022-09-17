package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  double Strafe_Speed;
  double Motor_Commands;
  double Turning_Degrees;

  AHRS Gyro;

  WPI_TalonFX Driving_Motor;
  WPI_TalonFX Turning_Motor;
  WPI_CANCoder Can_Coder;

  ProfiledPIDController TurningPID = new ProfiledPIDController(
    DriveConstants.Turning_P,
    0,
    0,
    new TrapezoidProfile.Constraints(6.28, 3.14)
  );


  PIDController DrivingPID = new PIDController(DriveConstants.Throttle_P, 0, 0);

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {
    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Turning_Motor = new WPI_TalonFX(turnMotorID);
    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);

    TurningPID.setTolerance(DriveConstants.Turning_Tolerance);
    DrivingPID.setTolerance(DriveConstants.Throttle_Tolerance);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(Strafe_Speed, calculateAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState,
      calculateAngle()
    );

    double driveOutput =
      state.speedMetersPerSecond /
      DriveConstants.Max_Strafe_Speed *
      DriveConstants.Max_Volts;
    double turnOutput = TurningPID.calculate(
      calculateAngle().getRadians(),
      state.angle.getRadians()
    );
    // Calculate the turning motor output from the turning PID controller.
    Driving_Motor.setVoltage(driveOutput);
    Turning_Motor.setVoltage(turnOutput);
  }

  public void resetEncoders() { //need to figure out offsets
    Driving_Motor.setSelectedSensorPosition(0);
    Turning_Motor.setSelectedSensorPosition(0);
  }

  public double getVelocity() {
    double speed =
      Driving_Motor.getSelectedSensorVelocity() *
      10 /
      DriveConstants.TalonFX_Encoder_Resolution /
      DriveConstants.Driving_Gearing *
      DriveConstants.Wheel_Circumference;
    return Units.feetToMeters(speed);
  }

  public Rotation2d calculateAngle() { // get angle from can coder
    return Rotation2d.fromDegrees(Can_Coder.getAbsolutePosition());
  }

  public double getRawEncoder(){
    return Can_Coder.getAbsolutePosition(); 
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
