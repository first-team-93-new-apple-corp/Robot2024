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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Preflight;
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
  // --------------------------------------------SUBSYSTEMS--------------------------------------------
  public final ShooterSubsystem m_ShooterSubsystem;
  public final IntakeSubsystem m_IntakeSubsystem;
  public final ElevatorSubsystem m_ElevatorSubsystem;
  public final ClimberSubsystem m_ClimberSubsystem;
  public final VisionSubsystem m_VisionSubsystem;
  public final Mechanisms m_MechanismsSubsystem;
  private final SwerveDriveSubsystem m_SwerveDriveSubsystem;
  public final LEDSubsystem m_LedSubsystem;
  // --------------------------------------------COMMANDS--------------------------------------------
  public final ShooterCommand m_ShooterCommand;
  public final IntakeCommand m_IntakeCommand;
  public final ElevatorCommand m_ElevatorCommand;  
  public final AutoAlignCommand m_AutoAlignCommand;  
  public final Preflight m_PreflightCommand;
  public final ClimberCommand m_ClimberCommand;
  // --------------------------------------------SHOOTER Control Buttons--------------------------------------------
  private final Trigger m_Prime;
  private final Trigger m_ShootAmp;
  private final Trigger m_IntakeFront;
  private final Trigger m_Kicker;
  private final Trigger m_KickerAmp;
  private final Trigger m_StopShooter;
  // --------------------------------------------DRIVE Buttons--------------------------------------------
  private final JoystickButton m_AmpAlignButton;
  private final JoystickButton m_BrakeButton;
  private final JoystickButton m_fieldRelButton;
  private final JoystickButton m_RobotRelButton;
  // --------------------------------------------SYS ID Buttons--------------------------------------------
  private final JoystickButton m_SysIDDriveQuasiButton;
  private final JoystickButton m_SysIDDriveDynamButton;
  private final JoystickButton m_SysIDSteerQuasiButton;
  private final JoystickButton m_SysIDSteerDynamButton;
  private final JoystickButton m_SysIDDriveSlipButton;
  private final JoystickButton m_endSignalLogging;

  private SwerveRequest.ApplyChassisSpeeds m_swerveRequest = new SwerveRequest.ApplyChassisSpeeds();
  private SendableChooser<Command> autoChooser;
  public final double MaxSpeed = DriveConstants.MaxSpeed;
  public final double MaxAngularRate = DriveConstants.MaxAngularRate;
  private double deadzone = DriveConstants.JoystickDeadzone;
  private Joystick m_Joystick1;
  private Joystick m_Joystick2;
  private XboxController op;
  private boolean Limit = true;
  public final EventLoop m_loop = new EventLoop();
  private ChassisSpeeds speeds;
  private ChassisSpeeds fieldSpeeds;
  private double fieldRelativeOffset;

  public Field2d m_Field2d = new Field2d();
  private Pose2d pose = new Pose2d();
  
  private SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov45.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FR
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov90.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Right
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov135.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BR
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov180.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Back
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov225.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_BL
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov270.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Left
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
              // .rotateBy(new Rotation2d(-fieldRelativeOffset))
              );
        } else if (pov315.getAsBoolean()) {
          DriveConstants.dCenter = DriveConstants.Location_FL
              .rotateBy(Rotation2d.fromDegrees(-1 * m_SwerveDriveSubsystem.getHeading())
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
    m_SwerveDriveSubsystem.registerTelemetry(logger::telemeterize);
    // --------------------------------------------DRIVE BUTTON BINDINGS--------------------------------------------
    m_AmpAlignButton.whileTrue(m_AutoAlignCommand.PathFindToAmp());

    m_SwerveDriveSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
        m_SwerveDriveSubsystem.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(ChassisSpeeds.discretize(fieldSpeeds, kDefaultPeriod))));

    m_fieldRelButton.onTrue(
        m_SwerveDriveSubsystem.applyRequest(() -> m_swerveRequest
            .withCenterOfRotation(DriveConstants.dCenter)
            .withSpeeds(ChassisSpeeds.discretize(fieldSpeeds, kDefaultPeriod))));

    // Brake while held
    m_BrakeButton.whileTrue(m_SwerveDriveSubsystem.applyRequest(() -> brake));

    m_RobotRelButton.onTrue(m_SwerveDriveSubsystem.applyRequest(() -> RobotCentricDrive
        .withVelocityX(
            -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)
                * MaxSpeed)
        .withVelocityY(
            -m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxSpeed)
        .withRotationalRate(
            -m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)
                * MaxAngularRate)));
    // --------------------------------------------SHOOTER BUTTON BINDINGS--------------------------------------------
    m_ShootAmp.whileTrue(m_ShooterCommand.ShootAmp());
    m_KickerAmp.whileTrue(m_ShooterCommand.AmpKicker().alongWith());
    m_Kicker.whileTrue(m_ShooterCommand.Kicker());
    m_IntakeFront.whileTrue(m_ShooterCommand.IntakeFront());
    m_Prime.whileTrue(m_ShooterCommand.Prime());
    //If not doing any of the above stop the shooter
    m_StopShooter.whileTrue(m_ShooterCommand.StopShooter());
    // --------------------------------------------SYS ID BUTTON BINDINGS--------------------------------------------
    m_SysIDDriveQuasiButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(m_SwerveDriveSubsystem.runDriveQuasiTest(Direction.kForward));
    m_SysIDDriveQuasiButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(m_SwerveDriveSubsystem.runDriveQuasiTest(Direction.kReverse));

    m_SysIDDriveDynamButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(m_SwerveDriveSubsystem.runDriveDynamTest(Direction.kForward));
    m_SysIDDriveDynamButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(m_SwerveDriveSubsystem.runDriveDynamTest(Direction.kReverse));

    m_SysIDSteerQuasiButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(m_SwerveDriveSubsystem.runSteerQuasiTest(Direction.kForward));
    m_SysIDSteerQuasiButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(m_SwerveDriveSubsystem.runSteerQuasiTest(Direction.kReverse));

    m_SysIDSteerDynamButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(m_SwerveDriveSubsystem.runSteerDynamTest(Direction.kForward));
    m_SysIDSteerDynamButton.and(m_Joystick1.pov(180, m_loop)).whileTrue(m_SwerveDriveSubsystem.runSteerDynamTest(Direction.kReverse));
    // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon wheel slip
    m_SysIDDriveSlipButton.and(m_Joystick1.pov(0, m_loop)).whileTrue(m_SwerveDriveSubsystem.runDriveSlipTest());
    m_endSignalLogging.whileTrue(m_SwerveDriveSubsystem.StopSignalLogging());
  }

  public RobotContainer(Constants constants, Joystick m_Joystick1, Joystick m_Joystick2, XboxController operator) {
    this.m_Joystick1 = m_Joystick1;
    this.m_Joystick2 = m_Joystick2;
    this.op = operator;
    // --------------------------------------------SUBSYSTEMS--------------------------------------------
    m_ShooterSubsystem = ShooterSubsystemFactory.build(constants.Shooter);
    m_LedSubsystem = new LEDSubsystem();
    m_SwerveDriveSubsystem = SwerveDriveSubsystemFactory.build(constants.Drive);
    m_IntakeSubsystem = IntakeSubsystemFactory.build(constants.Intake,m_LedSubsystem, m_ShooterSubsystem, op);
    m_ElevatorSubsystem = ElevatorSubsystemFactory.build(constants.Elevator);
    m_ClimberSubsystem = ClimberSubsystemFactory.build(constants.Climber);
    m_VisionSubsystem = VisionSubsystemFactory.build(m_SwerveDriveSubsystem, constants.Vision);
    m_MechanismsSubsystem = new Mechanisms(m_ElevatorSubsystem, m_ClimberSubsystem);
    // --------------------------------------------COMMANDS--------------------------------------------
    m_AutoAlignCommand = new AutoAlignCommand(m_SwerveDriveSubsystem, m_Joystick1);
    m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem, m_LedSubsystem);
    m_IntakeCommand = new IntakeCommand(m_ShooterSubsystem, m_IntakeSubsystem, m_LedSubsystem);
    m_ElevatorCommand = new ElevatorCommand(op, m_ElevatorSubsystem);
    m_ClimberCommand = new ClimberCommand(op, m_ClimberSubsystem);
    m_PreflightCommand = new Preflight(m_ElevatorCommand, m_ClimberCommand);
    // --------------------------------------------Shooter Control Buttons--------------------------------------------
    m_ShootAmp = new Trigger(op.button(Constants.xbox.RightShoulderButton, m_loop));
    m_KickerAmp = new JoystickButton(m_Joystick2, Constants.Thrustmaster.Trigger).and(op.button(Constants.xbox.RightShoulderButton, m_loop));  
    m_Kicker = new JoystickButton(m_Joystick2, Constants.Thrustmaster.Trigger).and(op.button(Constants.xbox.RightShoulderButton, m_loop).negate());  
    m_IntakeFront = new Trigger(op.button(Constants.xbox.LeftShoulderButton, m_loop));
    m_Prime = new Trigger(op.axisGreaterThan(Constants.xbox.Axis.RT, 0.6, m_loop));
    m_StopShooter = new Trigger((m_ShootAmp.and(m_KickerAmp).and(m_Kicker).and(m_IntakeFront).and(m_Prime)).negate());
    // --------------------------------------------DRIVE Control Buttons--------------------------------------------
    m_fieldRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Top_Middle);
    m_BrakeButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Trigger);
    m_RobotRelButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Left_Buttons.Bottom_Middle);
    m_AmpAlignButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Center_Button);
    // --------------------------------------------SYS ID BUTTONS--------------------------------------------
    m_SysIDDriveQuasiButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Top_Left);
    m_SysIDDriveDynamButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Top_Middle);
    m_SysIDSteerQuasiButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Bottom_Left);
    m_SysIDSteerDynamButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Bottom_Middle);
    m_SysIDDriveSlipButton = new JoystickButton(m_Joystick1, Constants.Thrustmaster.Right_Buttons.Top_Right);
    m_endSignalLogging = new JoystickButton(m_Joystick2, Constants.Thrustmaster.Right_Buttons.Bottom_Right);
    // --------------------------------------------PATH PLANNER NAMED COMMANDS--------------------------------------------
    NamedCommands.registerCommand("Intake", m_IntakeCommand.AutoIntake().alongWith(Commands.waitSeconds(1.5)).andThen(m_IntakeCommand.AutonStopIntake()));
    NamedCommands.registerCommand("Shooter", m_ShooterCommand.AutonShooter().alongWith(Commands.waitSeconds(.5)).andThen(m_ShooterCommand.AutonStopShooter()));
    NamedCommands.registerCommand("StopShooter", m_ShooterCommand.AutonStopShooter());
    NamedCommands.registerCommand("ShootAmp", m_ShooterCommand.AutonAmp());
    NamedCommands.registerCommand("DribbleNote", m_ShooterCommand.AutonDribbleNote());
    NamedCommands.registerCommand("StopIntake", m_IntakeCommand.AutonStopIntake());
    NamedCommands.registerCommand("ResetField", m_SwerveDriveSubsystem.resetPigeonAuton());
    // --------------------------------------------OTHER/MISC--------------------------------------------
    SignalLogger.start();
    if (Utils.isSimulation()) {
      m_SwerveDriveSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    m_SwerveDriveSubsystem.configAuto();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Field",m_Field2d);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public void updateVision() {
    m_SwerveDriveSubsystem.updateOdometry(m_VisionSubsystem);
    pose = m_SwerveDriveSubsystem.m_SwerveDrivePoseEstimator.getEstimatedPosition();
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
    if (m_Joystick1.getRawButtonPressed(Constants.Thrustmaster.Left_Buttons.Top_Middle)) {
      fieldRelativeOffset = m_SwerveDriveSubsystem.getPigeon2().getRotation2d().getRadians();
      m_SwerveDriveSubsystem.getPigeon2().reset();
    }
    speeds = new ChassisSpeeds(
        ((checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y)) * Math.abs(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.y))) * MaxSpeed),
        (checkDeadzone(-m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * Math.abs(m_Joystick1.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxSpeed),
        (checkDeadzone(-m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * Math.abs(m_Joystick2.getRawAxis(Constants.Thrustmaster.Axis.x)) * MaxAngularRate));
    fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
        new Rotation2d(m_SwerveDriveSubsystem.getPigeon2().getRotation2d().getRadians())
            // .rotateBy(new Rotation2d(-fieldRelativeOffset))
            );
    RotationPoints(m_Joystick2);
    POVButton();
    updateVision();
    // --------------------------------------------SMARTDASHBOARD STUFF--------------------------------------------
    SmartDashboard.putData("pigeon", getPigeon());
    SmartDashboard.putNumber("angular Velocity", getPigeon().getRate());
    // --------------------------------------------SYS ID LOGGING--------------------------------------------
    SignalLogger.writeDoubleArray("pose", new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    SignalLogger.writeDouble("overall X pose", pose.getX());
    SignalLogger.writeDouble("Stator Current", m_SwerveDriveSubsystem.getDrive(0).getStatorCurrent().getValueAsDouble());
    for (int i = 0; i < 4; i++) {
    SignalLogger.writeDouble("SysID: Stator Current" + i, m_SwerveDriveSubsystem.getDrive(i).getStatorCurrent().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Drive Velocity" + i, m_SwerveDriveSubsystem.getDrive(i).getVelocity().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Drive Position" + i, m_SwerveDriveSubsystem.getDrive(i).getPosition().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Drive Voltage" + i, m_SwerveDriveSubsystem.getDrive(i).getMotorVoltage().getValueAsDouble());
    
    SignalLogger.writeDouble("SysID: Steer Velocity" + i, m_SwerveDriveSubsystem.getTurn(i).getVelocity().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Steer Position" + i, m_SwerveDriveSubsystem.getTurn(i).getPosition().getValueAsDouble());
    SignalLogger.writeDouble("SysID: Steer Voltage" + i, m_SwerveDriveSubsystem.getTurn(i).getMotorVoltage().getValueAsDouble());
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
    return m_SwerveDriveSubsystem;
  }

  public Pigeon2 getPigeon() {
    return m_SwerveDriveSubsystem.getPigeon2();
  }
}
