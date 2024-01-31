package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoShootStates.RobotStates;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.RobotContainer;


public class AutoShootSubsystem extends SubsystemBase{
    Joystick joystick = new Joystick(0);
    NetworkTable networkTable;
    double tx;
    double ty;
    double ta;
    double tv;
    double aid; // april tag id
    double speakerID = 3;
    double targetdistance = 23;
    double speakerHeight = 100;
    double limelightAngle = 45;
    public static final double MaxSpeed = DriveConstants.MaxSpeed;
    public final double MaxAngularRate = DriveConstants.MaxAngularRate;
    ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public SwerveDriveSubsystem m_drivetrain;
    double setpoint = 0;
    PIDController AutoShootPID = new PIDController(0.1, 0, 0.1);
    
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    public AutoShootSubsystem(SwerveDriveSubsystem driveTrain) {
        this.m_drivetrain = driveTrain;
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // public boolean hasTargets() {
    //     return tv ==1;
    // }
    public void alignToSpeaker() {
        if (aid == speakerID) {
            m_drivetrain.setControl(drive.withVelocityX(AutoShootPID.calculate(tx, setpoint)));
            if (tx >5) {
                System.out.println("Outside range");
                
            }
            // if (tx >-5 && tx <5 && calculateDistance()> targetdistance) {
            //     System.out.println("Outside target distance");
            //     m_drivetrain.setControl(drive.withVelocityY(AutoShootPID.calculate(calculateDistance(), targetdistance)));
            // }
            m_drivetrain.setControl(brake);
        }

    }
    public void shootSpeaker() {
        shooterSubsystem.ShooterR.set(shooterSubsystem.SpeakerShooterSpeed);
        shooterSubsystem.ShooterL.set(-shooterSubsystem.SpeakerShooterSpeed);
    }
    public double calculateDistance() {
        return speakerHeight/Math.tan(limelightAngle);
    }
    public void periodic() {
        tx = networkTable.getValue("tx").getDouble();
        ty = networkTable.getValue("ty").getDouble();
        ta = networkTable.getValue("ta").getDouble();
        tv = networkTable.getValue("tv").getDouble();
        aid  = networkTable.getValue("tid").getDouble();
        
        if (Constants.AutoShootStates.RobotState.equals(RobotStates.AUTOSHOOT)) {
            alignToSpeaker();
            if (tx >-5 && tx <5 && calculateDistance() < targetdistance) {
                shootSpeaker();
                // Constants.AutoShootStates.RobotState = RobotStates.TELEOP;
            }
        }
    }
}
