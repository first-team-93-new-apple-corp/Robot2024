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

  AHRS Gyro;
  SwerveDriveOdometry Odometry;
  SwerveDriveKinematics Kinematics;

  SwerveModule Front_Left;
  SwerveModule Front_Right;
  SwerveModule Back_Left;
  SwerveModule Back_Right;
  Trajectory Trajectory;

  ProfiledPIDController Turning_PID = new ProfiledPIDController(
    DriveConstants.TurningP,
    0,
    0,
    new TrapezoidProfile.Constraints(6.28, 3.14)
  );
  PIDController Driving_PID_X = new PIDController(1, 0, 0);
  PIDController Driving_PID_Y = new PIDController(1, 0, 0);

  ChassisSpeeds Speeds;
  HolonomicDriveController Controller;

  public DriveSubsystem() {
    Gyro = new AHRS(SPI.Port.kMXP);

    Kinematics =
      new SwerveDriveKinematics(
        DriveConstants.locationFL,
        DriveConstants.locationFR,
        DriveConstants.locationBL,
        DriveConstants.locationBR
      );
    Odometry = new SwerveDriveOdometry(Kinematics, Gyro.getRotation2d());

    Front_Left =
      new SwerveModule(
        DriveConstants.FLThrottlePort,
        DriveConstants.FLTurningPort,
        DriveConstants.FLEncoderPort,
        DriveConstants.FLMagnetOffset
      );
    Front_Right =
      new SwerveModule(
        DriveConstants.FRThrottlePort,
        DriveConstants.FRTurningPort,
        DriveConstants.FREncoderPort,
        DriveConstants.FRMagnetOffset
      );
    Back_Left =
      new SwerveModule(
        DriveConstants.BLThrottlePort,
        DriveConstants.BLTurningPort,
        DriveConstants.BLEncoderPort,
        DriveConstants.BLMagnetOffset
      );
    Back_Right =
      new SwerveModule(
        DriveConstants.BRThrottlePort,
        DriveConstants.BRTurningPort,
        DriveConstants.BREncoderPort,
        DriveConstants.BRMagnetOffset
      );

    Controller =
      new HolonomicDriveController(Driving_PID_X, Driving_PID_Y, Turning_PID);
  }

  public void drive(double X, double Y, double Z, boolean Field_Relative) { // from joystick
    if (!Field_Relative) {
      Speeds =
        new ChassisSpeeds(
          X * DriveConstants.maxStrafeSpeed,
          Y * DriveConstants.maxStrafeSpeed,
          Z * DriveConstants.maxAngularSpeed
        );
    } else {
      Speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          X * DriveConstants.maxStrafeSpeed,
          Y * DriveConstants.maxStrafeSpeed,
          Z * DriveConstants.maxAngularSpeed,
          Gyro.getRotation2d()
        );
    }

    SwerveModuleState[] states = Kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      DriveConstants.maxStrafeSpeed
    );

    Front_Left.setDesiredState(states[0]);
    Front_Right.setDesiredState(states[1]);
    Back_Left.setDesiredState(states[2]);
    Back_Right.setDesiredState(states[3]);
  }

  public void drive_Auton(double time) {
    // we are passing in time from the command, this could then get passed into the
    // sample
    Trajectory.State goal = Trajectory.sample(time);
    ChassisSpeeds adjustedSpeeds = Controller.calculate(
      getPose(),
      goal,
      Rotation2d.fromDegrees(0)
    );
    SwerveModuleState[] states = Kinematics.toSwerveModuleStates(
      adjustedSpeeds
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      DriveConstants.maxStrafeSpeed
    );

    Front_Left.setDesiredState(states[0]);
    Front_Right.setDesiredState(states[1]);
    Back_Left.setDesiredState(states[2]);
    Back_Right.setDesiredState(states[3]);
  }

  public void getStates() {
    Front_Left.getState();
    Front_Right.getState();
    Back_Left.getState();
    Back_Right.getState();
  }

  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  public double getHeading() {
    return Gyro.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    Gyro.reset();
  }

  public void resetEncoders() {
    Front_Left.resetEncoders();
    Front_Right.resetEncoders();
    Back_Left.resetEncoders();
    Back_Right.resetEncoders();
  }

  public void resetOdometry(Pose2d pose) {
    Odometry.resetPosition(pose, Gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    Odometry.update(
      Gyro.getRotation2d(),
      Front_Left.getState(),
      Front_Right.getState(),
      Back_Left.getState(),
      Back_Right.getState()
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
