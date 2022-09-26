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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    DriveConstants.Turning_P,
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
        DriveConstants.Location_FL,
        DriveConstants.Location_FR,
        DriveConstants.Location_BL,
        DriveConstants.Location_BR
      );

    Odometry = new SwerveDriveOdometry(Kinematics, Gyro.getRotation2d());

    // Setting Up Swerve Modules
    Front_Left =
      new SwerveModule(
        DriveConstants.Throttle_Port_FL,
        DriveConstants.Turning_Port_FL,
        DriveConstants.Encoder_Port_FL,
        DriveConstants.Magnet_Offset_FL
      );
    // Front_Right =
    //   new SwerveModule(
    //     DriveConstants.Throttle_Port_FR,
    //     DriveConstants.Turning_Port_FR,
    //     DriveConstants.Encoder_Port_FR,
    //     DriveConstants.Magnet_Offset_FR
    //   );
    // Back_Left =
    //   new SwerveModule(
    //     DriveConstants.Throttle_Port_BL,
    //     DriveConstants.Turning_Port_BL,
    //     DriveConstants.Encoder_Port_BL,
    //     DriveConstants.Magnet_Offset_BL
    //   );
    // Back_Right =
    //   new SwerveModule(
    //     DriveConstants.Throttle_Port_BR,
    //     DriveConstants.Turning_Port_BR,
    //     DriveConstants.Encoder_Port_BR,
    //     DriveConstants.Magnet_Offset_BR
    //   );

    // Swerve Drive PID 
    Controller =
      new HolonomicDriveController(Driving_PID_X, Driving_PID_Y, Turning_PID);

      SmartDashboard.putNumber("Swerve Module Angle", 0);
  }

  // public void getEncoderValues(){
  //   System.out.println("Front Right: " + Front_Right.calculateAngle());
  //   System.out.println("Front Left: " + Front_Left.calculateAngle());
  //   System.out.println("Back Right: " + Back_Right.calculateAngle());
  //   System.out.println("Back Left: " + Back_Left.calculateAngle());
  // }

  public void drive(double X, double Y, double Z){ //, boolean Field_Relative) { // from joystick
    
    // setting up speeds based on whether field relative is on or not
    // passing in joystick values in params

    // if (!Field_Relative) {
      Speeds =
        new ChassisSpeeds(
          X * DriveConstants.Max_Strafe_Speed,
          Y * DriveConstants.Max_Strafe_Speed,
          Z * DriveConstants.Max_Angular_Speed
        );

    // } 
    // else {
    //   Speeds =
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //       X * DriveConstants.Max_Strafe_Speed,
    //       Y * DriveConstants.Max_Strafe_Speed,
    //       Z * DriveConstants.Max_Angular_Speed,
    //       Gyro.getRotation2d()
    //     );
    // }

    // Swerve module states 
    SwerveModuleState[] States = Kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      States,
      DriveConstants.Max_Angular_Speed
    );

    // setting states 
  SwerveModuleState testing =  new SwerveModuleState(0.2, new Rotation2d(SmartDashboard.getNumber("Swerve Module Angle", 0)));
    Front_Left.setDesiredState(testing);
    // Front_Right.setDesiredState(States[1]);
    // Back_Left.setDesiredState(States[2]);
    // Back_Right.setDesiredState(States[3]);
  }

  // public void driveAuton(double time) {
  //   // we are passing in time from the command, this could then get passed into the
  //   // sample
  //   Trajectory.State goal = Trajectory.sample(time);
  //   ChassisSpeeds adjustedSpeeds = Controller.calculate(
  //     getPose(),
  //     goal,
  //     Rotation2d.fromDegrees(0)
  //   );
  //   SwerveModuleState[] states = Kinematics.toSwerveModuleStates(
  //     adjustedSpeeds
  //   );

  //   SwerveDriveKinematics.desaturateWheelSpeeds(
  //     states,
  //     DriveConstants.Max_Angular_Speed
  //   );

  //   Front_Left.setDesiredState(states[0]);
  //   Front_Right.setDesiredState(states[1]);
  //   Back_Left.setDesiredState(states[2]);
  //   Back_Right.setDesiredState(states[3]);
  // }

  public void getStates() {
    Front_Left.getState();
    // Front_Right.getState();
    // Back_Left.getState();
    // Back_Right.getState();
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
    // Front_Right.resetEncoders();
    // Back_Left.resetEncoders();
    // Back_Right.resetEncoders();
  }

  public void resetOdometry(Pose2d pose) {
    Odometry.resetPosition(pose, Gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Odometry.update(
    //   Gyro.getRotation2d(),
    //   Front_Left.getState(),
    //   Front_Right.getState(),
    //   Back_Left.getState(),
    //   Back_Right.getState()
    // );

    SmartDashboard.putNumber("Actual Angle", Front_Left.calculateAngle().getDegrees());
    SmartDashboard.putNumber("Actual Angle", Front_Right.calculateAngle().getDegrees());
    SmartDashboard.putNumber("Actual Angle", Back_Left.calculateAngle().getDegrees());
    SmartDashboard.putNumber("Actual Angle", Back_Right.calculateAngle().getDegrees());
  }

  @Override
  public void simulationPeriodic() {}
}
