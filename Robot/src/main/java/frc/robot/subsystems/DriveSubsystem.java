package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.StopDriveCommand;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveSubsystem extends SubsystemBase {

  Pigeon2 Pigeon;
  SwerveDriveOdometry Odometry;
  SwerveDriveKinematics Kinematics;

  SwerveModule Front_Left;
  SwerveModule Front_Right;
  SwerveModule Back_Left;
  SwerveModule Back_Right;
  Trajectory Trajectory;
  Rotation2d getRotation2d;
  HolonomicDriveController AutoController;
  PIDController XYPIDController;
  ProfiledPIDController ThetaController;

  SwerveModuleState SavedStates[];

  ChassisSpeeds Speeds;

  enum DriveState {
    DEFAULT_STATE,
    HELD_FIELD_RELATIVE,
    TOGGLE_FIELD_RELATIVE_STAGE_1,
    TOGGLE_FIELD_RELATIVE_STAGE_2,
    TOGGLE_HOLD_STATE,
  }

  private DriveState CurrentDriveState = DriveState.DEFAULT_STATE;

  public DriveSubsystem() {

    SmartDashboard.putBoolean("Field Relative", false);
    SmartDashboard.putString("Current Drive State", CurrentDriveState.name());

    Pigeon = new Pigeon2(0);
    Pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
    Pigeon.setYaw(0);

    getRotation2d = Rotation2d.fromDegrees(Pigeon.getYaw());

    Kinematics = new SwerveDriveKinematics(
        DriveConstants.Location_FL,
        DriveConstants.Location_FR,
        DriveConstants.Location_BL,
        DriveConstants.Location_BR);

    Odometry = new SwerveDriveOdometry(Kinematics, getRotation2d);

    // Setting Up Swerve Modules
    Front_Left = new SwerveModule(
        DriveConstants.Throttle_Port_FL,
        DriveConstants.Turning_Port_FL,
        DriveConstants.Encoder_Port_FL,
        DriveConstants.Magnet_Offset_FL);
    Front_Right = new SwerveModule(
        DriveConstants.Throttle_Port_FR,
        DriveConstants.Turning_Port_FR,
        DriveConstants.Encoder_Port_FR,
        DriveConstants.Magnet_Offset_FR);
    Back_Left = new SwerveModule(
        DriveConstants.Throttle_Port_BL,
        DriveConstants.Turning_Port_BL,
        DriveConstants.Encoder_Port_BL,
        DriveConstants.Magnet_Offset_BL);
    Back_Right = new SwerveModule(
        DriveConstants.Throttle_Port_BR,
        DriveConstants.Turning_Port_BR,
        DriveConstants.Encoder_Port_BR,
        DriveConstants.Magnet_Offset_BR);

    XYPIDController = new PIDController(1, 0, 0);
    Constraints constraints = new Constraints(Math.PI, Math.PI);
    ThetaController = new ProfiledPIDController(Math.PI / 2, 0, 0, constraints);
    AutoController = new HolonomicDriveController(XYPIDController, XYPIDController, ThetaController);
    SmartDashboard.putNumber("Passing in angle to state", 0);
  }

  public void setModuleStates(SwerveModuleState[] States) {
    SavedStates = States; 
    Front_Left.setDesiredState(States[0]);
    Front_Right.setDesiredState(States[1]);
    Back_Left.setDesiredState(States[2]);
    Back_Right.setDesiredState(States[3]);
  }

  public void stopMotors(){
    drive(0,0,0, false, DriveConstants.Center);
  }
  public void drive(double X, double Y, double Z, boolean Field_Relative, Translation2d COR) { // from joystick
    // setting up speeds based on whether field relative is on or not
    // passing in joystick values in params

    if (!Field_Relative) {
      Speeds = new ChassisSpeeds(
          X * DriveConstants.Max_Strafe_Speed,
          Y * DriveConstants.Max_Strafe_Speed,
          Z * DriveConstants.Max_Angular_Speed);
    } else {
      Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          X * DriveConstants.Max_Strafe_Speed,
          Y * DriveConstants.Max_Strafe_Speed,
          Z * DriveConstants.Max_Angular_Speed,
          Rotation2d.fromDegrees(getHeading()));
    }

    // Swerve module states
    SwerveModuleState[] States = Kinematics.toSwerveModuleStates(
        Speeds,
        COR);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        States,
        DriveConstants.Max_Angular_Speed);
    // this is so that the angle is saved and not auto set to 0 even after strafe
    if (X == 0 && Y == 0 && Z == 0) {
      if (SavedStates == null) {
        SavedStates = States;
      }
      for (int i = 0; i < SavedStates.length; i++) {
        States[i].speedMetersPerSecond = 0;
        States[i].angle = SavedStates[i].angle;
      }
    } else {
      SavedStates = States;
    }

    Front_Left.setDesiredState(States[0]);
    Front_Right.setDesiredState(States[1]);
    Back_Left.setDesiredState(States[2]);
    Back_Right.setDesiredState(States[3]);
  }

  public void resetDriveMode() {
    CurrentDriveState = DriveState.DEFAULT_STATE;
  }

  public void DriveStateMachine(
      double x,
      double y,
      double z,
      Boolean HeldButton,
      Boolean HeldButtonReleased,
      Boolean ToggleButton,
      Boolean ToggleButtonReleased,
      Translation2d Rotation,
      double MaxSpeedMultipier) {

//configuring speed multiplier; 
        x=x*MaxSpeedMultipier;
        y=y*MaxSpeedMultipier;
        z=z*MaxSpeedMultipier;

           // Setting up Starting State or simple driving

    SmartDashboard.putString("Current Drive State", CurrentDriveState.name());

    if (CurrentDriveState == DriveState.DEFAULT_STATE) {
      // if the toggle button is pressed

      if (ToggleButton) {
        SmartDashboard.putBoolean("Field Relative", true);
        zeroHeading();
        CurrentDriveState = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_1;
      }
      // if the held button is pressed
      else if (HeldButton) {
        SmartDashboard.putBoolean("Field Relative", true);
        zeroHeading();
        CurrentDriveState = DriveState.HELD_FIELD_RELATIVE;
      } else {
        SmartDashboard.putBoolean("Field Relative", false);
        drive(x, y, z, false, Rotation);
      }
    }

    // running held and toggle field modes
    switch (CurrentDriveState) {
      // keep field relative drive until button is released
      case HELD_FIELD_RELATIVE:
        drive(x, y, z, true, Rotation);
        if (HeldButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      // waiting for button to be released
      case TOGGLE_FIELD_RELATIVE_STAGE_1:
        drive(x, y, z, true, Rotation);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
        }
        break;
      // waiting for button to be pressed again
      case TOGGLE_FIELD_RELATIVE_STAGE_2:
        drive(x, y, z, true, Rotation);
        if (ToggleButton) {
          CurrentDriveState = DriveState.TOGGLE_HOLD_STATE;
        }
        break;

      // waiting for button 12 to be released, stay in this mode until it has been
      // released, but run non-field relative drive in the meantime
      case TOGGLE_HOLD_STATE:
        SmartDashboard.putBoolean("Field Relative", false);
        drive(x, y, z, false, Rotation);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      default:
        break;
    }
  }

  // Assuming this method is part of a drivetrain subsystem that provides the
  // necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            this.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            this.Kinematics, // SwerveDriveKinematics
            new PIDController(1, 0, 0), // if overtuned robot will never stop
            new PIDController(1, 0, 0), // if overtuned, robot will never stop
            new PIDController(1, 0, 0), // if undertuned, robot will not reach chassis heading
            this::setModuleStates, // Module states consumer
            this // Requires this drive subsystem
        ),

        new StopDriveCommand(this)); 
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
    double yaw = Pigeon.getYaw();
    yaw = yaw % 360;
    return yaw;
  }

  public void zeroHeading() {
    Pigeon.setYaw(0);
  }

  public void resetOdometry(Pose2d pose) {
    System.out.println(pose);
    zeroHeading();
    Odometry.resetPosition(pose, pose.getRotation());
  }

  DecimalFormat round = new DecimalFormat("#.###");

  public void printEncoderValues() {
    SmartDashboard.putString("Front Right Encoder", round.format(Front_Right.getAngle().getDegrees()));
    SmartDashboard.putString("Front Left Encoder", round.format(Front_Left.getAngle().getDegrees()));
    SmartDashboard.putString("Back Right Encoder", round.format(Back_Right.getAngle().getDegrees()));
    SmartDashboard.putString("Back Left Encoder", round.format(Back_Left.getAngle().getDegrees()));

  }

  public void TryingAccelerometer() {
    // double[] GravVec = new double[3];
    // System.out.println(Pigeon.getGravityVector(GravVec));
    // System.out.println(GravVec[0]);
    // double[] QuaternionOutput = new double[4];
    // System.out.println(QuaternionOutput[2]);
    // System.out.println(Pigeon.get6dQuaternion(QuaternionOutput));
    // System.out.println(QuaternionOutput[2]);
    short[] AccelerometerData = new short[3];
    Pigeon.getBiasedAccelerometer(AccelerometerData);
    double gsX = AccelerometerData[0] / 16384.;

    double accelerationX = gsX / 9.8;
    SmartDashboard.putNumber("X Acceleration", gsX);

  }

  @Override
  public void periodic() {
    TryingAccelerometer();

    printEncoderValues();

    Odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        Front_Left.getState(),
        Front_Right.getState(),
        Back_Left.getState(),
        Back_Right.getState());

  }

  @Override
  public void simulationPeriodic() {
  }
}
