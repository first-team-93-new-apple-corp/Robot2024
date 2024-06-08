package frc.robot.DriveInputs;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve.CollisionDeteaction;

public class TwoStickDriveCollisions extends TwoStickExponentialDrive {
    Supplier<Pose2d> pose;
    CollisionDeteaction collisionDeteaction = new CollisionDeteaction();
    public TwoStickDriveCollisions(int Joystick1Port, int Joystick2Port, Supplier<Pose2d> pose) {
        super(Joystick1Port, Joystick2Port);
        this.pose = pose;
    }

    @Override
    public ChassisSpeeds fieldSpeeds(Rotation2d Robotangle){
        return collisionDeteaction.CollisionSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeeds(), Robotangle), pose.get() ); 
    }    
}
