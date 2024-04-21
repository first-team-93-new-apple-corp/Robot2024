// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystemFactory;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystemFactory;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystemFactory;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystemFactory;
import frc.robot.subsystems.Swerve.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystemFactory;
import frc.robot.subsystems.Swerve.Telemetry;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystemFactory;

public class RobotContainer extends TimedRobot {

  // public ClimbingLevel m_ClimbingLevel;
  public ShooterCommand m_ShooterCommand;
  public IntakeCommand m_IntakeCommand;
  public ElevatorCommand m_ElevatorCommand;  
  public ShooterSubsystem m_ShooterSubsystem;
  public IntakeSubsystem m_IntakeSubsystem;
  public ElevatorSubsystem m_ElevatorSubsystem;
  public ClimberSubsystem m_ClimberSubsystem;
  public VisionSubsystem m_VisionSubsystem;
  public Mechanisms m_Mechanisms;
  private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private final SwerveDriveSubsystem drivetrain; // My drivetrain
  private SendableChooser<Command> autoChooser;
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private double deadzone = DriveConstants.JoystickDeadzone;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;
  @SuppressWarnings("unused")
  private XboxController op;
  private boolean Limit = true;
  // can set this to whatever button you want you can also just
  // delete this and use the constants file for the button
  // (just so that the logic works for now)
  // private int climbingLevelButton;
  public final EventLoop m_loop = new EventLoop();
  private ChassisSpeeds speeds;
  private ChassisSpeeds fieldSpeeds;
  private double fieldRelativeOffset;
  JoystickButton m_AmpAlignButton;
  JoystickButton m_TrapAlignButton;
  private final JoystickButton m_BrakeButton;
  private final JoystickButton m_fieldRelButton;
  private final JoystickButton m_RobotRelButton;

  private final JoystickButton m_CameraRelButton;

  private final JoystickButton m_SysIDDriveQuasiButton;
  private final JoystickButton m_SysIDDriveDynamButton;
  private final JoystickButton m_SysIDSteerQuasiButton;
  private final JoystickButton m_SysIDSteerDynamButton;
  private final JoystickButton m_SysIDDriveSlipButton;
  private final JoystickButton m_endSignalLogging;
  public Field2d m_Field2d = new Field2d();
  private Pose2d pose = new Pose2d();
  // private final SwerveDrivePoseEstimator m_poseEstimator;
  // added this for button bindings and the logic I added
  
  AutoAlignCommand m_AutoAlignCommand;

  private SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  POVButton pov0; // front
  POVButton pov45; // fr wheel
  POVButton pov90; // right
  POVButton pov135; // br wheel
  POVButton pov180; // back
  POVButton pov225;// bl wheel
  POVButton pov270;// left
  POVButton pov315; // fl wheel
  POVButton povCenter;

  // logan was here you silly gooses ;)
  // Configures the bindings to drive / control the swerve drive :)
  public void RotationPoints(Joystick m_Joystick2) {
    m_Joystick2 = this.m_Joystick2;
    pov0 = new POVButton(m_Joystick2, 0); // front
    pov45 = new POVButton(m_Joystick2, 45); // fr wheel
    pov90 = new POVButton(m_Joystick2, 90); // right
    pov135 = new POVButton(m_Joystick2, 135); // br wheel
    pov180 = new POVButton(m_Joystick2, 180); // back
    pov225 = new POVButton(m_Joystick2, 225);// bl wheel
    pov270 = new POVButton(m_Joystick2, 270);// left
    pov315 = new POVButton(m_Joystick2, 315); // fl wheel
    povCenter = new POVButton(m_Joystick2, -1);
  }

  public void POVButton() {
    // SmartDashboard.putNumber("Limit", Limit);
    if (!(m_Joystick2.getRawAxis(0) <= 0.1 && m_Joystick2.getRawAxis(0) >= -0.1)) {
      if (Limit) {
        Limit = false;
        // SmartDashboard.putNumber("Limit", Limit);
        if (pov0.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Front
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov45.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FR
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov90.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Right
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov135.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BR
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov180.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Back
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov225.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BL
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov270.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Left
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov315.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FL
              .rotateBy(Rotation2d.fromDegrees(-1 * drivetrain.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else {
          DriveConstants.dCenter = DriveConstants.Center;
        }
      } else {
        Limit = false;
        if (povCenter.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Center;
        }
      }
    } else {
      Limit = true;
    }
  }
  public void configureBindings() {
    m_TrapAlignButton.whileTrue(m_AutoAlignCommand);
    m_AmpAlignButton.whileTrue(m_AutoAlignCommand.PathFindToAmp());
    m_endSignalLogging.whileTrue(drivetrain.StopSignalLogging());
    // m_AmpAlignButton.whileTrue(drivetrain.applyRequest(() -> m_swerveRequest
    // .withCenterOfRotation(DriveConstants.dCenter)
    // .withSpeeds(m_AutoAlignSubsystem.fieldSpeeds)));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(fieldSpeeds)));

    m_fieldRelButton.onTrue(
        drivetrain.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(fieldSpeeds)));

    // Brake while held
    m_BrakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
    m_RobotRelButton.onTrue(drivetrain.applyRequest(() -> RobotCentricDrive
        .withVelocityX(
            -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
                * MaxSpeed)
        .withVelocityY(
            -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxSpeed)
        .withRotationalRate(
            -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxAngularRate)));

    m_CameraRelButton.whileTrue(drivetrain.applyRequest(() -> RobotCentricDrive
        .withVelocityX(
            m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
                * MaxSpeed)
        .withVelocityY(
            m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxSpeed)
        .withRotationalRate(
            -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxAngularRate)));

    // Climbing Level logic added to button
    // When the button of your choosing is held it should atomaticly do the climbing
    // level
    // m_climbingLevelButton.whileTrue(m_ClimbingLevel);

    // Points all in a direction
    // m_WheelsPointForwardButton.whileTrue(drivetrain
    //     .applyRequest(
    //         () -> point.withModuleDirection(new Rotation2d(-m_Joystick1.getRawAxis(0),
    //             -m_Joystick1.getRawAxis(1)))));

    // reset the field-centric heading on left bumper press

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    
    m_SysIDDriveQuasiButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    m_SysIDDriveQuasiButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    m_SysIDDriveDynamButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    m_SysIDDriveDynamButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    m_SysIDSteerQuasiButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kForward));
    m_SysIDSteerQuasiButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(drivetrain.runSteerQuasiTest(Direction.kReverse));

    m_SysIDSteerDynamButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(drivetrain.runSteerDynamTest(Direction.kForward));
    m_SysIDSteerDynamButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(drivetrain.runSteerDynamTest(Direction.kReverse));
    // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon wheel slip
    m_SysIDDriveSlipButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(drivetrain.runDriveSlipTest());

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // public void configAuto() {
  //   drivetrain.configAuto();
  // }
  public Pose3d getTagPose(AprilTag tag){
    return tag.pose;
  }

  public RobotContainer(Constants constants, Joystick m_Joystick1, Joystick m_Joystick2, XboxController op, LEDSubsystem m_LedSubsystem) {
    this.m_ShooterSubsystem = ShooterSubsystemFactory.build(constants.Shooter);
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.op = op;
    drivetrain = SwerveDriveSubsystemFactory.build(constants.Drive);
    m_IntakeSubsystem = IntakeSubsystemFactory.build(constants.Intake,m_LedSubsystem, m_ShooterSubsystem, op);
    m_ElevatorSubsystem = ElevatorSubsystemFactory.build(constants.Elevator);
    m_ClimberSubsystem = ClimberSubsystemFactory.build(constants.Climber);
    m_VisionSubsystem = VisionSubsystemFactory.build(drivetrain, constants.Vision);
    m_IntakeCommand = new IntakeCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_LedSubsystem);
    m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem);


    SmartDashboard.putData("Field",m_Field2d);

    m_Mechanisms = new Mechanisms(m_ElevatorSubsystem, m_ClimberSubsystem);

    m_AutoAlignCommand = new AutoAlignCommand(drivetrain, m_Joystick1);
    m_fieldRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Top_Middle);
    m_BrakeButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    // m_WheelsPointForwardButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Center_Button);
    m_RobotRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Bottom_Middle);
    m_CameraRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    m_AmpAlignButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Center_Button);
    m_TrapAlignButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Button);

    m_SysIDDriveQuasiButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Top_Left);
    m_SysIDDriveDynamButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Top_Middle);
    m_SysIDSteerQuasiButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Bottom_Left);
    m_SysIDSteerDynamButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Bottom_Middle);
    m_SysIDDriveSlipButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Top_Right);
    m_endSignalLogging = new JoystickButton(m_Joystick2, Constants.Thrustmaster.Right_Buttons.Bottom_Right);

    //ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    //IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem(m_ShooterSubsystem);
    // m_AutoAlignSubsystem = new AutoAlignSubsystem(drivetrain);
    // m_climbingLevelButton = new JoystickButton(op, climbingLevelButton);
    NamedCommands.registerCommand("Intake", m_IntakeCommand.AutoIntake().alongWith(Commands.waitSeconds(1.5)).andThen(m_IntakeCommand.AutonStopIntake()));
    NamedCommands.registerCommand("Shooter", m_ShooterCommand.AutonShooter().alongWith(Commands.waitSeconds(.5)).andThen(m_ShooterCommand.AutonStopShooter()));
    NamedCommands.registerCommand("StopShooter", m_ShooterCommand.AutonStopShooter());
    NamedCommands.registerCommand("ShootAmp", m_ShooterCommand.AutonAmp());
    NamedCommands.registerCommand("DribbleNote", m_ShooterCommand.AutonDribbleNote());
    // NamedCommands.registerCommand("StopKicker", m_ShooterSubsystem.AutonKickerStop());
    NamedCommands.registerCommand("StopIntake", m_IntakeCommand.AutonStopIntake());
    NamedCommands.registerCommand("ResetField", drivetrain.resetPigeonAuton());
    SignalLogger.start();
    drivetrain.configAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public void updateVision() {
    drivetrain.updateOdometry(m_VisionSubsystem);
    pose = drivetrain.m_SwerveDrivePoseEstimator.getEstimatedPosition();
    m_Field2d.setRobotPose(pose);
  }

  private List<Pose2d> poses;
  public void pathTrajectory() {
    try {
      poses =
      PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName()).get(0).getAllPathPoints().stream()
        .map(point -> new Pose2d(point.position, new Rotation2d()))
        .collect(Collectors.toList());
        m_Field2d.getObject("Traj").setTrajectory(TrajectoryGenerator.generateTrajectory(poses, new TrajectoryConfig(3, 3)));
    } catch (Exception e) {
    }
  }

  public void updateValues() {
    m_Mechanisms.periodic();
    if (m_Joystick1.getRawButtonPressed(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
      fieldRelativeOffset = drivetrain.getPigeon2().getRotation2d().getRadians();
      drivetrain.getPigeon2().reset();
    }
    speeds = new ChassisSpeeds(
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)) * MaxSpeed),
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxSpeed),
        (checkDeadzone(-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxAngularRate));
    fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        new Rotation2d(drivetrain.getPigeon2().getRotation2d().getRadians())
            // .rotateBy(new Rotation2d(-fieldRelativeOffset))
            );
    RotationPoints(m_Joystick2);
    POVButton();
    // m_AutoAlignSubsystem.Alliance();
    updateVision();

    SmartDashboard.putData("pigeon", getPigeon());
    SmartDashboard.putNumber("angular Velocity", getPigeon().getRate());

    SignalLogger.writeDoubleArray("pose", new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    SignalLogger.writeDouble("overall X pose", pose.getX());
    SignalLogger.writeDouble("test", drivetrain.getDrive(0).getStatorCurrent().getValueAsDouble());
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
    
        builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getModule(0).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getModule(0).getCurrentState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getModule(1).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getModule(1).getCurrentState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(2).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getModule(2).getCurrentState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getModule(3).getCurrentState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getModule(3).getCurrentState().speedMetersPerSecond, null);
    
        builder.addDoubleProperty("Robot Angle", () -> getPigeon().getAngle(), null);
      }
    });
    for (int i = 0; i < 4; i++) {
    SmartDashboard.putNumber("Drive Module Voltages" + i,drivetrain.getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Module Speeds" + i,drivetrain.getModule(i).getCurrentState().speedMetersPerSecond);

    SignalLogger.writeDouble("SysID: Stator Current" + i, drivetrain.getDrive(i).getStatorCurrent().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Drive Velocity" + i, drivetrain.getDrive(i).getVelocity().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Drive Position" + i, drivetrain.getDrive(i).getPosition().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Drive Voltage" + i, drivetrain.getDrive(i).getMotorVoltage().getValueAsDouble());
    
    SignalLogger.writeDouble("SysID: Steer Velocity" + i, drivetrain.getTurn(i).getVelocity().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Steer Position" + i, drivetrain.getTurn(i).getPosition().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Steer Voltage" + i, drivetrain.getTurn(i).getMotorVoltage().getValueAsDouble());
    }
  }

  public double checkDeadzone(double input) {
    if (deadzone > input && input > -deadzone) {
      return 0;
    } else {
      return input;
    }
  }

  public SwerveDriveSubsystem getDrive() {
    return drivetrain;
  }

  public Pigeon2 getPigeon() {
    return drivetrain.getPigeon2();
  }
}
