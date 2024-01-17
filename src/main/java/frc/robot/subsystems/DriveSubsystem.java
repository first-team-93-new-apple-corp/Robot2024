package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.text.DecimalFormat;

public class DriveSubsystem extends SubsystemBase {

  public static Pigeon2 Pigeon;

  public static SwerveModule Front_Left;
  public static SwerveModule Front_Right;
  public static SwerveModule Back_Left;
  public static SwerveModule Back_Right;

  SwerveModuleState SavedStates[];

  public static SwerveDriveOdometry Odometry;

  SwerveDriveKinematics Kinematics;

  Rotation2d InitialRotation2d;

  ChassisSpeeds Speeds;

  static SwerveDrivePoseEstimator poseEstimator;

  public enum DriveState {
    DEFAULT_STATE,
    HELD_FIELD_RELATIVE,
    TOGGLE_FIELD_RELATIVE_STAGE_1,
    TOGGLE_FIELD_RELATIVE_STAGE_2,
    TOGGLE_HOLD_STATE,
  }

  public DriveState CurrentDriveState = DriveState.DEFAULT_STATE;

  private SwerveModuleState[] LockWheelState;

  public double Starting_Level;

  public DriveSubsystem() {

    SmartDashboard.putBoolean("Field Relative", false);
    SmartDashboard.putString("Current Drive State", CurrentDriveState.name());

    Pigeon = new Pigeon2(0);
    Pigeon.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ);
    Pigeon.setYaw(0);
    // Pigeon.configMountPosePitch(1);
    // Pigeon.setPitch(0);
    Starting_Level = Pigeon.getPitch();

    LockWheelState = new SwerveModuleState[4];

    // Front Left
    LockWheelState[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    //Front Right
    LockWheelState[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    //Back Left
    LockWheelState[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    //Back Right
    LockWheelState[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    InitialRotation2d = Rotation2d.fromDegrees(Pigeon.getYaw());

    Kinematics =
      new SwerveDriveKinematics(
        Constants.Drive.Location_FL,
        Constants.Drive.Location_FR,
        Constants.Drive.Location_BL,
        Constants.Drive.Location_BR
      );

    SmartDashboard.putNumber("Pigeon Angle", getHeading());
    // Setting Up Swerve Modules
    Front_Left =
      new SwerveModule(
        Constants.Drive.Throttle_Port_FL,
        Constants.Drive.Turning_Port_FL,
        Constants.Drive.Encoder_Port_FL,
        Constants.Drive.Magnet_Offset_FL
      );

    Front_Right =
      new SwerveModule(
        Constants.Drive.Throttle_Port_FR,
        Constants.Drive.Turning_Port_FR,
        Constants.Drive.Encoder_Port_FR,
        Constants.Drive.Magnet_Offset_FR
      );

    Back_Left =
      new SwerveModule(
        Constants.Drive.Throttle_Port_BL,
        Constants.Drive.Turning_Port_BL,
        Constants.Drive.Encoder_Port_BL,
        Constants.Drive.Magnet_Offset_BL
      );

    Back_Right =
      new SwerveModule(
        Constants.Drive.Throttle_Port_BR,
        Constants.Drive.Turning_Port_BR,
        Constants.Drive.Encoder_Port_BR,
        Constants.Drive.Magnet_Offset_BR
      );

    Odometry =
      new SwerveDriveOdometry(Kinematics, InitialRotation2d, getPositions());
    resetOdometry(new Pose2d());
  }

  public void lockWheels() {
    SavedStates = LockWheelState;
    Front_Left.setDesiredState(LockWheelState[0]);
    Front_Right.setDesiredState(LockWheelState[1]);
    Back_Left.setDesiredState(LockWheelState[2]);
    Back_Right.setDesiredState(LockWheelState[3]);
  }

  public void setModuleStates(SwerveModuleState[] States) {
    SavedStates = States;
    Front_Left.setDesiredState(States[0]);
    Front_Right.setDesiredState(States[1]);
    Back_Left.setDesiredState(States[2]);
    Back_Right.setDesiredState(States[3]);
  }

  public void stopMotors() {
    drive(0, 0, 0, false, DriveConstants.dCenter);
  }

  public void drive(
    double X,
    double Y,
    double Z,
    boolean Field_Relative,
    Translation2d COR
  ) {
    if (!Field_Relative) {
      Speeds =
        new ChassisSpeeds(
          X * Constants.Drive.Max_Strafe_Speed,
          Y * Constants.Drive.Max_Strafe_Speed,
          Z * Constants.Drive.Max_Angular_Speed
        );
    } else {
      Speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          X * Constants.Drive.Max_Strafe_Speed,
          Y * Constants.Drive.Max_Strafe_Speed,
          Z * Constants.Drive.Max_Angular_Speed,
          Rotation2d.fromDegrees(getHeading())
        );
    }
    // Swerve module states
    SwerveModuleState[] States = Kinematics.toSwerveModuleStates(Speeds, COR);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      States,
      Constants.Drive.Max_Strafe_Speed
    );

    // this is so that the angle is saved and not auto set to 0 even after strafe
    if (X == 0 && Y == 0 && Z == 0) {
      if (SavedStates == null) {
        SavedStates = States;
      }
      for (int i = 0; i < SavedStates.length; i++) {
        States[i].speedMetersPerSecond = 0;
        States[i].angle = SavedStates[i].angle;
      }
    }

    setModuleStates(States);
  }

  public void resetDriveStateMachine() {
    CurrentDriveState = DriveState.DEFAULT_STATE;
  }

  public void DriveStateMachine(
    double x,
    double y,
    double z,
    Boolean HeldButton,
    Boolean HeldButtonReleased,
    Boolean ToggleButton,
    Boolean ToggleButtonReleased
  ) {
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
        drive(x, y, z, false, DriveConstants.dCenter);
      }
    }

    // running held and toggle field modes
    switch (CurrentDriveState) {
      // keep field relative drive until button is released
      case HELD_FIELD_RELATIVE:
        drive(x, y, z, true, DriveConstants.dCenter);
        if (HeldButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      // waiting for button to be released
      case TOGGLE_FIELD_RELATIVE_STAGE_1:
        drive(x, y, z, true, DriveConstants.dCenter);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
        }
        break;
      // waiting for button to be pressed again
      case TOGGLE_FIELD_RELATIVE_STAGE_2:
        drive(x, y, z, true, DriveConstants.dCenter);
        if (ToggleButton) {
          CurrentDriveState = DriveState.TOGGLE_HOLD_STATE;
        }
        break;
      // waiting for button to be released, stay in this mode until it has been
      // released, but run non-field relative drive in the meantime
      case TOGGLE_HOLD_STATE:
        SmartDashboard.putBoolean("Field Relative", false);
        drive(x, y, z, false, DriveConstants.dCenter);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      default:
        break;
    }
  }

  public static SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = {
      Front_Left.getPosition(),
      Front_Right.getPosition(),
      Back_Left.getPosition(),
      Back_Right.getPosition(),
    };

    return positions;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = {
      Front_Left.getState(),
      Front_Right.getState(),
      Back_Left.getState(),
      Back_Right.getState(),
    };

    return states;
  }

  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  public static double getHeading() {
    double yaw = Pigeon.getYaw();
    yaw = yaw % 360;
    if (yaw < 0) {
      yaw = 360 + yaw;
    }
    return yaw;
  }

  public static void zeroHeading() {
    Pigeon.setYaw(0);
  }

  public void resetOdometry(Pose2d pose) {
    System.out.println("INTIAL POSE");
    System.out.println(pose + "\n");

    Pigeon.setYaw(pose.getRotation().getDegrees());
    Odometry.resetPosition(pose.getRotation(), getPositions(), pose);
  }

  public double getLevel() {
    return Pigeon.getPitch();
  }

  DecimalFormat round = new DecimalFormat("#.###");

  public double GetAcceleration() {
    short[] fill = { 0, 0, 0 };
    Pigeon.getBiasedAccelerometer(fill);
    return (double) fill[1] / 16384. * 9.8;
  }

  public void printEncoderValues() {
    SmartDashboard.putString(
      "Front Right Encoder",
      round.format(Front_Right.getAngle().getDegrees())
    );
    SmartDashboard.putString(
      "Front Left Encoder",
      round.format(Front_Left.getAngle().getDegrees())
    );
    SmartDashboard.putString(
      "Back Right Encoder",
      round.format(Back_Right.getAngle().getDegrees())
    );
    SmartDashboard.putString(
      "Back Left Encoder",
      round.format(Back_Left.getAngle().getDegrees())
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon Angle", getHeading());
    // SmartDashboard.putNumber("Front Left Turning Motor Temp", Front_Left.TurningTemp());
    // SmartDashboard.putNumber("Front Left Driving Motor Temp", Front_Left.DriveTemp());
    // SmartDashboard.putNumber("Front Right Turning Motor Temp", Front_Right.TurningTemp());
    // SmartDashboard.putNumber("Front Right Driving Motor Temp", Front_Right.DriveTemp());
    // SmartDashboard.putNumber("Back Left Turning Motor Temp", Back_Left.TurningTemp());
    // SmartDashboard.putNumber("Back Left Driving Motor Temp", Back_Left.DriveTemp());
    // SmartDashboard.putNumber("Back Right Turning Motor Temp", Back_Right.TurningTemp());
    // SmartDashboard.putNumber("Back Right Driving Motor Temp", Back_Right.DriveTemp());
    SmartDashboard.putNumber("AccelerationX", GetAcceleration());
    // these values should be uncommented when zeroing encoders
    // DO NOT use the ones that show up in shuffleboard, those are sus and not
    // accurate
    // See Sameer for more info
    SmartDashboard.putNumber(
      "Front Right Cancoder",
      Front_Right.getCancoderTicks()
    );
    SmartDashboard.putNumber(
      "Front Left Cancoder",
      Front_Left.getCancoderTicks()
    );
    SmartDashboard.putNumber(
      "Back Right Cancoder",
      Back_Right.getCancoderTicks()
    );
    SmartDashboard.putNumber(
      "Back Left Cancoder",
      Back_Left.getCancoderTicks()
    );

    Odometry.update(Rotation2d.fromDegrees(getHeading()), getPositions());
    SmartDashboard.putString("Odometry", getPose().toString());
    SmartDashboard.putNumber("Level", getLevel());
  }

  @Override
  public void simulationPeriodic() {}
}
