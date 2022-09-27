package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

  // ProfiledPIDController TurningPID = new ProfiledPIDController(
  //   DriveConstants.Turning_P,
  //   DriveConstants.Turning_I,
  //   DriveConstants.Turning_D,
  //   new TrapezoidProfile.Constraints(0.5, 0.15)
  // );

  PIDController TurningPID = new PIDController(
    DriveConstants.Turning_P,
    DriveConstants.Turning_I,
    DriveConstants.Turning_D
  );
  

  // PIDController DrivingPID = new PIDController(DriveConstants.Throttle_P, 0, 0);

  public SwerveModule(
    int driveMotorID,
    int turnMotorID,
    int CanCoderID,
    double magnetOffset
  ) {
  // TurningPID.setIntegratorRange(-0.1,0.1);
    Driving_Motor = new WPI_TalonFX(driveMotorID);
    Turning_Motor = new WPI_TalonFX(turnMotorID);
    // Turning_Motor.setInverted(true);
    Turning_Motor.setNeutralMode(NeutralMode.Brake);

    Can_Coder = new WPI_CANCoder(CanCoderID);
    Can_Coder.configMagnetOffset(magnetOffset);

    TurningPID.setTolerance(DriveConstants.Turning_Tolerance);

    SmartDashboard.putNumber("Turning P", 0);
    SmartDashboard.putNumber("Turning I", 0);
    SmartDashboard.putNumber("Turning D", 0);
    SmartDashboard.putBoolean("At Setpoint", TurningPID.atSetpoint());
    // DrivingPID.setTolerance(DriveConstants.Throttle_Tolerance);
  }

  public SwerveModuleState getState() {
    // Swerve module states contain a speed and wheel angle
    // should Strafe_Speed be replaced with get velocity?
    // replaced, but check with Henry later

    return new SwerveModuleState(getVelocity(), calculateAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // optimize which way to turn the wheel
    SwerveModuleState state = SwerveModuleState.optimize(
      desiredState,
      calculateAngle()
    );
    // SwerveModuleState state = desiredState;

    // System.out.println(state);
    // System.out.println(state.angle.getRadians());
    // System.out.println(calculateAngle().getRadians());
    // System.out.println(calculateAngle().getDegrees());

    //do not need PID on drive motors - just a simple voltage calculation
    // double driveOutput =
    //   (state.speedMetersPerSecond / DriveConstants.Max_Strafe_Speed) *
    //   DriveConstants.Max_Volts;

    //Turning needs a pid because it has a setpoint it need to reach
    double turnOutput = TurningPID.calculate(
      calculateAngle().getRadians(),
      state.angle.getRadians()
    );

    // System.out.println(turnOutput);
    SmartDashboard.putNumber("Set Angle in Radians", state.angle.getRadians());
    SmartDashboard.putNumber("Turn Output", turnOutput);
    SmartDashboard.putBoolean("At Setpoint", TurningPID.atSetpoint());

    // Setting voltage based on motor calculations
    // Driving_Motor.setVoltage(driveOutput);
    Turning_Motor.setVoltage(turnOutput);
    // Turning_Motor.set(0.04);
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

  // get raw encoder ticks for testing
  public double getRawEncoder() {
    return Can_Coder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    TurningPID.setP(SmartDashboard.getNumber("Turning P", 0));
    TurningPID.setI(SmartDashboard.getNumber("Turning I", 0));
    TurningPID.setD(SmartDashboard.getNumber("Turning D", 0));
  }

  @Override
  public void simulationPeriodic() {}
}
