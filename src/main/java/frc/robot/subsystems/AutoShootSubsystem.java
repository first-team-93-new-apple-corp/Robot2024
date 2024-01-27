package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem;
public class AutoShootSubsystem extends SubsystemBase{
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
    ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    
    public AutoShootSubsystem() {
        networkTable = NetworkTableInstance.getDefault().getTable("Pipeline_Name");
    }
    // public boolean hasTargets() {
    //     return tv ==1;
    // }
    public void alignSpeaker() {
        if (aid == speakerID) {
            if (tx <-5) {
                
            }
            if (tx >5) {

            }
            if (calculateDistance()> targetdistance ) {
                
            }
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

    }
}
