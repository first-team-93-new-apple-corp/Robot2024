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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.text.DecimalFormat;

public class DriveSubsystem extends SubsystemBase {

  Pigeon2 Pigeon;

  SwerveModule Front_Left;
  SwerveModule Front_Right;
  SwerveModule Back_Left;
  SwerveModule Back_Right;

  SwerveModuleState SavedStates[];

  SwerveDriveOdometry Odometry;

  SwerveDriveKinematics Kinematics;

  Rotation2d getRotation2d;

  ChassisSpeeds Speeds;

  static SwerveDrivePoseEstimator poseEstimator;

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
    Pigeon.configMountPose(AxisDirection.NegativeY, AxisDirection.PositiveZ);
    Pigeon.setYaw(0);

    getRotation2d = Rotation2d.fromDegrees(Pigeon.getYaw());

    Kinematics =
      new SwerveDriveKinematics(
        DriveConstants.Location_FL,
        DriveConstants.Location_FR,
        DriveConstants.Location_BL,
        DriveConstants.Location_BR
      );

    Odometry = new SwerveDriveOdometry(Kinematics, getRotation2d);

    SmartDashboard.putNumber("sus", Pigeon.getYaw());
    // Setting Up Swerve Modules
    Front_Left =
      new SwerveModule(
        DriveConstants.Throttle_Port_FL,
        DriveConstants.Turning_Port_FL,
        DriveConstants.Encoder_Port_FL,
        DriveConstants.Magnet_Offset_FL
      );

    Front_Right =
      new SwerveModule(
        DriveConstants.Throttle_Port_FR,
        DriveConstants.Turning_Port_FR,
        DriveConstants.Encoder_Port_FR,
        DriveConstants.Magnet_Offset_FR
      );

    Back_Left =
      new SwerveModule(
        DriveConstants.Throttle_Port_BL,
        DriveConstants.Turning_Port_BL,
        DriveConstants.Encoder_Port_BL,
        DriveConstants.Magnet_Offset_BL
      );

    Back_Right =
      new SwerveModule(
        DriveConstants.Throttle_Port_BR,
        DriveConstants.Turning_Port_BR,
        DriveConstants.Encoder_Port_BR,
        DriveConstants.Magnet_Offset_BR
      );

  }

  public void setModuleStates(SwerveModuleState[] States) {
    SavedStates = States;
    Front_Left.setDesiredState(States[0]);
    Front_Right.setDesiredState(States[1]);
    Back_Left.setDesiredState(States[2]);
    Back_Right.setDesiredState(States[3]);
  }

  public void stopMotors() {
    drive(0, 0, 0, false, DriveConstants.Center);
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
          X * DriveConstants.Max_Strafe_Speed,
          Y * DriveConstants.Max_Strafe_Speed,
          Z * DriveConstants.Max_Angular_Speed
        );
    } else {
      Speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          X * DriveConstants.Max_Strafe_Speed,
          Y * DriveConstants.Max_Strafe_Speed,
          Z * DriveConstants.Max_Angular_Speed,
          Rotation2d.fromDegrees(getHeading())
        );
    }
    // Swerve module states
    SwerveModuleState[] States = Kinematics.toSwerveModuleStates(Speeds, COR);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      States,
      DriveConstants.Max_Strafe_Speed
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
    Boolean ToggleButtonReleased,
    Translation2d Rotation
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
        drive(x, y, z, false, DriveConstants.Center);
      }
    }

    // running held and toggle field modes
    switch (CurrentDriveState) {
      // keep field relative drive until button is released
      case HELD_FIELD_RELATIVE:
        drive(x, y, z, true, DriveConstants.Center);
        if (HeldButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      // waiting for button to be released
      case TOGGLE_FIELD_RELATIVE_STAGE_1:
        drive(x, y, z, true, DriveConstants.Center);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_2;
        }
        break;
      // waiting for button to be pressed again
      case TOGGLE_FIELD_RELATIVE_STAGE_2:
        drive(x, y, z, true, DriveConstants.Center);
        if (ToggleButton) {
          CurrentDriveState = DriveState.TOGGLE_HOLD_STATE;
        }
        break;
      // waiting for button to be released, stay in this mode until it has been
      // released, but run non-field relative drive in the meantime
      case TOGGLE_HOLD_STATE:
        SmartDashboard.putBoolean("Field Relative", false);
        drive(x, y, z, false, DriveConstants.Center);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      default:
        break;
    }
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
    Pigeon.setYaw(pose.getRotation().getDegrees());
    Odometry.resetPosition(pose, pose.getRotation());
  }

  DecimalFormat round = new DecimalFormat("#.###");

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
    SmartDashboard.putNumber("sus", Pigeon.getYaw());

    Odometry.update(Rotation2d.fromDegrees(getHeading()), getStates());
  }

  @Override
  public void simulationPeriodic() {}
}
