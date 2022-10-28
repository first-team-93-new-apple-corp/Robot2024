package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    SmartDashboard.putBoolean("Field Relative" , false);
    SmartDashboard.putString("Current Drive State", CurrentDriveState.name());

    Pigeon = new Pigeon2(0);
    Pigeon.configMountPose(AxisDirection.PositiveX, AxisDirection.PositiveZ);
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

    SmartDashboard.putNumber("Passing in angle to state", 0);
  }

  public void drive(double X, double Y, double Z, boolean Field_Relative) { // from joystick
    // setting up speeds based on whether field relative is on or not
    // passing in joystick values in params

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
    SwerveModuleState[] States = Kinematics.toSwerveModuleStates(
      Speeds,
      DriveConstants.Center
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      States,
      DriveConstants.Max_Angular_Speed
    );
    //this is so that the angle is saved and not auto set to 0 even after strafe
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



  public void resetDriveMode(){
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
    // Setting up Starting State or simple driving

    SmartDashboard.putString("Current Drive State", CurrentDriveState.name());

    if (CurrentDriveState == DriveState.DEFAULT_STATE) {
      // if the toggle button is pressed

      if (ToggleButton) {
        SmartDashboard.putBoolean("Field Relative" , true);
        zeroHeading();
        CurrentDriveState = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_1;
      }
      // if the held button is pressed
      else if (HeldButton) {
        SmartDashboard.putBoolean("Field Relative" , true);
        zeroHeading();
        CurrentDriveState = DriveState.HELD_FIELD_RELATIVE;
      } else {
        SmartDashboard.putBoolean("Field Relative" , false);
        drive(x, y, z, false);
      }
    }

    // running held and toggle field modes
    switch (CurrentDriveState) {
      // keep field relative drive until button is released
      case HELD_FIELD_RELATIVE:
        ;
        drive(x, y, z, true);
        if (HeldButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      // waiting for button to be released
      case TOGGLE_FIELD_RELATIVE_STAGE_1:
        drive(x, y, z, true);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.TOGGLE_FIELD_RELATIVE_STAGE_1;
        }
        break;
      // waiting for button to be pressed again
      case TOGGLE_FIELD_RELATIVE_STAGE_2:
        drive(x, y, z, true);
        if (ToggleButton) {
          CurrentDriveState = DriveState.TOGGLE_HOLD_STATE;
        }
        break;

      // waiting for button 12 to be released, stay in this mode until it has been released, but run non-field relative drive in the meantime
      case TOGGLE_HOLD_STATE:
        SmartDashboard.putBoolean("Field Relative" , false);
        drive(x, y, z, false);
        if (ToggleButtonReleased) {
          CurrentDriveState = DriveState.DEFAULT_STATE;
        }
        break;
      default:
        break;
    }
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
    Odometry.resetPosition(pose, getRotation2d);
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

  }

  @Override
  public void simulationPeriodic() {}
}
