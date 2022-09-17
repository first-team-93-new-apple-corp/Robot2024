package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  AHRS gyro;
  SwerveDriveOdometry odometry;
  SwerveDriveKinematics kinematics;

  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;
  Trajectory trajectory;

  ProfiledPIDController TurningPID = new ProfiledPIDController(SwerveDriveConstantsFalcon.TurningP, 0, 0,
      new TrapezoidProfile.Constraints(6.28, 3.14));
  PIDController DrivingPIDX = new PIDController(1, 0, 0);
  PIDController DrivingPIDY = new PIDController(1, 0, 0);

  ChassisSpeeds Speeds;
  HolonomicDriveController controller;

  public Drivetrain() {
    gyro = new AHRS(SPI.Port.kMXP);

    kinematics = new SwerveDriveKinematics(SwerveDriveConstantsFalcon.locationFL, SwerveDriveConstantsFalcon.locationFR,
        SwerveDriveConstantsFalcon.locationBL, SwerveDriveConstantsFalcon.locationBR);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

    frontLeft = new SwerveModule(SwerveDriveConstantsFalcon.FLThrottlePort,
        SwerveDriveConstantsFalcon.FLTurningPort, SwerveDriveConstantsFalcon.FLEncoderPort,SwerveDriveConstantsFalcon.FLMagnetOffset);
    frontRight = new SwerveModule(SwerveDriveConstantsFalcon.FRThrottlePort,
        SwerveDriveConstantsFalcon.FRTurningPort, SwerveDriveConstantsFalcon.FREncoderPort,SwerveDriveConstantsFalcon.FRMagnetOffset);
    backLeft = new SwerveModule(SwerveDriveConstantsFalcon.BLThrottlePort,
        SwerveDriveConstantsFalcon.BLTurningPort, SwerveDriveConstantsFalcon.BLEncoderPort,SwerveDriveConstantsFalcon.BLMagnetOffset);
    backRight = new SwerveModule(SwerveDriveConstantsFalcon.BRThrottlePort,
        SwerveDriveConstantsFalcon.BRTurningPort, SwerveDriveConstantsFalcon.BREncoderPort,SwerveDriveConstantsFalcon.BRMagnetOffset);

    controller = new HolonomicDriveController(DrivingPIDX, DrivingPIDY, TurningPID); 
  }

  public void drive(double X, double Y, double Z, boolean Fieldrelative) { // from joystick
    if (!Fieldrelative) {
      Speeds = new ChassisSpeeds(X * SwerveDriveConstantsFalcon.maxStrafeSpeed,
          Y * SwerveDriveConstantsFalcon.maxStrafeSpeed, Z * SwerveDriveConstantsFalcon.maxAngularSpeed);
    } else {
      Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(X * SwerveDriveConstantsFalcon.maxStrafeSpeed,
          Y * SwerveDriveConstantsFalcon.maxStrafeSpeed, Z * SwerveDriveConstantsFalcon.maxAngularSpeed,
          gyro.getRotation2d());
    }

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDriveConstantsFalcon.maxStrafeSpeed);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);

  }

  public void driveAuton(double time) {
    // we are passing in time from the command, this could then get passed into the
    // sample
    Trajectory.State goal = trajectory.sample(time);
    ChassisSpeeds adjustedSpeeds = controller.calculate(getPose(), goal, Rotation2d.fromDegrees(0));
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(adjustedSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDriveConstantsFalcon.maxStrafeSpeed);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);

  }

  public void getStates() {

    frontLeft.getState();
    frontRight.getState();
    backLeft.getState();
    backRight.getState();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        backRight.getState());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}