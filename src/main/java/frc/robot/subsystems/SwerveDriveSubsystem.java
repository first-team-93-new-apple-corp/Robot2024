package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystemFactory;

public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
    public XboxController opController = new XboxController(2);
    // public ShooterCommand m_ShooterCommand = new ShooterCommand();
    // public ShooterSubsystem m_shooter = new ShooterSubsystem();
    // public IntakeCommand m_IntakeCommand = new IntakeCommand(m_shooter);
    // public ElevatorCommand m_ElevatorCommand = new ElevatorCommand(opController);
    public final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Telemetry m_Telemetry = new Telemetry(MaxSpeed);
    public SwerveDrivePoseEstimator m_SwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            m_kinematics,
            m_pigeon2.getRotation2d(),
            m_modulePositions,
            new Pose2d());
    public VisionSubsystem m_VisionSubsystem = VisionSubsystemFactory.build(this);
    public void configAuto() {
        // AutoBuilder.configureHolonomic(
        // this::getPose, // Robot pose supplier
        // this::resetPose, // Method to reset odometry (will be called if your auto has
        // // a starting pose)
        // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
        // // RELATIVE
        // this::driveRobotRelative, // Method that will drive the robot given ROBOT
        // // RELATIVE ChassisSpeeds
        // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
        // // likely live in your
        // // Constants class
        // new PIDConstants(0.25, 0.0, 0.0), // Translation PID constants
        // new PIDConstants(0.25, 0.0, 0.0), // Rotation PID constants
        // 4.5, // Max module speed, in m/s
        // 0.4, // Drive base radius in meters. Distance from robot center to furthest
        // // module.
        // new ReplanningConfig() // Default path replanning config. See the API for the
        // // options here
        // ),
        // () -> {
        // // Boolean supplier that controls when the path will be mirrored for the red
        // // alliance
        // // This will flip the path being followed to the red side of the field.
        // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        // return alliance.get() == DriverStation.Alliance.Red;
        // }
        // return false;
        // },
        // this // Reference to this subsystem to set requirements
        // );
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        
        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                // () -> this.m_SwerveDrivePoseEstimator.getEstimatedPosition(),
                this::resetOdometry, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(5.5
                , 0.1, 0),
                        new PIDConstants(1.4, 0, .8),
                        // new HolonomicPathFollowerConfig(new PIDConstants(.1, 0, 0),
                        // new PIDConstants(.25, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,                                                                                        //These values are deadzones, so to speak
                        new ReplanningConfig(true, true)),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false; // Change this if the path needs to be flipped on red vs blue
                },
                this); // Subsystem for requirements
    }

    public void resetOdometry(Pose2d Pose) {
        seedFieldRelative(Pose);
        m_SwerveDrivePoseEstimator.resetPosition(new Rotation2d(Math.toRadians(getHeading())), m_modulePositions, Pose);
        
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void applyConfig(TalonFXConfiguration config, TalonFXConfigurator configurator) {

    }

    public TalonFX getTurn(int module) {
        return Modules[module].getSteerMotor();
    }

    public TalonFX getDrive(int module) {
        return Modules[module].getDriveMotor();
    }

    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {            startSimThread();
        }
    }

    public double getHeading() {
        double yaw = m_pigeon2.getYaw().getValueAsDouble();
        yaw = yaw % 360;
        if (yaw < 0) {
            yaw = 360 + yaw;
        }
        return yaw;
    }
    public Command resetPigeonAuton() {
        return this.runOnce(() -> m_pigeon2.reset());
    }
    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public SwerveModulePosition[] getModulePositions () {
        return m_modulePositions;
    }

    public void updateOdometry() {
        m_SwerveDrivePoseEstimator.update(m_pigeon2.getRotation2d(), m_modulePositions);
        try {
            if (m_VisionSubsystem.hasTargets()) {
                m_SwerveDrivePoseEstimator.addVisionMeasurement(m_VisionSubsystem.getPose(),
                        Timer.getFPGATimestamp() - (m_VisionSubsystem.getLatency()));
            }
        } catch (Exception e) {
        }
        
    }

    public Pose2d getPose() {
        // return m_cachedState.Pose;
        return m_odometry.getEstimatedPosition();
    }

    public void setPose2D(Pose2d newPose) {
        m_cachedState.Pose = newPose;
    }

    public SwerveDrivePoseEstimator getOdometry() {
        return m_odometry;
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public void resetPose(Pose2d pose) {
        m_Telemetry.updatePose(pose);
        getState().Pose = pose;
    }
    
    private double X,Y,Theta,tofpos,tofposAngle,calculatedX, calculatedY, calculatedTheta;
    private PIDController AlignPIDTheta = new PIDController(.25, 0, 0.13); // Rotationly PID
    private PIDController AlignPIDX = new PIDController(3.25, 2, 0.4); // Drive PIDs should be the same
    private PIDController AlignPIDY = new PIDController(2, 1.5, 0);
    private PIDController tofPID = new PIDController(0.008, 0, 0.00125);
    private SlewRateLimiter xlimit = new SlewRateLimiter(12);
    private SlewRateLimiter ylimit = new SlewRateLimiter(12);
    private ChassisSpeeds alignSpeeds; // Chassis Speeds which robot uses for auto align
    private ChassisSpeeds AlignfieldSpeeds;
    private TimeOfFlight tof = new TimeOfFlight(23);

    public void toPose(Pose2d pose) {
        
        updateOdometry();
        X = m_SwerveDrivePoseEstimator.getEstimatedPosition().getX();
        Y = m_SwerveDrivePoseEstimator.getEstimatedPosition().getY();
        Theta = m_SwerveDrivePoseEstimator.getEstimatedPosition().getRotation().getRadians();
        tofpos = tof.getRange();
        if (Theta > 90) {
            tofposAngle = 90-(Theta-90);
        } else {
            tofposAngle = Theta;
        }
        tofpos = tofpos*Math.sin(tofposAngle);
        calculatedX = xlimit
                .calculate((MathUtil.clamp((AlignPIDX.calculate(X, pose.getX())), -MaxSpeed, MaxSpeed)) * 1);

        calculatedTheta = (MathUtil.clamp((AlignPIDTheta.calculate(Theta, (pose.getRotation().getRadians()))),
                -MaxAngularRate, MaxAngularRate));
        if (Utils.isSimulation()) {
            calculatedY = ylimit.calculate((MathUtil.clamp((AlignPIDY.calculate(Y, pose.getY())), -MaxSpeed, MaxSpeed)) * 1);
        } else {
            calculatedY = MathUtil.clamp(-tofPID.calculate(tofpos, 293), -MaxSpeed, MaxSpeed);
        }
        alignSpeeds = new ChassisSpeeds(calculatedX, calculatedY, calculatedTheta);

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                // red
                AlignfieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(Math.PI)));
            } else {
                // Blue
                AlignfieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(alignSpeeds,
                new Rotation2d(getPigeon2().getRotation2d().getRadians())
                        .rotateBy(new Rotation2d(0)));
            }
        }
        driveRobotRelative(AlignfieldSpeeds);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(m_cachedState.ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Before leaving DS write what your current goal so someone else can take over
        // if needed
        // applyRequest(() -> )
        setControl(robotDrive.withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {

        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        this.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}