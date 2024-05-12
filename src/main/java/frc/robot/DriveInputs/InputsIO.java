package frc.robot.DriveInputs;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve.DriveConstants;
import frc.robot.subsystems.Swerve.SwerveDriveSubsystem;

public interface InputsIO {

    /**
    * Returns a object containing all our inputs
    */
    public Pose2d Inputs();

    /**
    * Returns a double containing our forward input
    */
    public default double ForwardInput(){
        return checkDeadzone(Inputs().getX());
    }

    /**
    * Returns a double containing our Sideways input
    */
    public default double SidewaysInput(){
        return checkDeadzone(Inputs().getY());
    }

    /**
    * Returns a double containing our Rotational input
    */
    public default double omegaInput(){
        return checkDeadzone(Inputs().getRotation().getRadians());
    }
    
    /**
    * Returns a double containing our forward Speed
    */
    public default double ForwardMetersPerSecond(){
        return ForwardInput() * DriveConstants.MaxSpeed;
    }

    /**
    * Returns a double containing our Sideways input
    */
    public default double SidewaysMetersPerSecond(){
        return SidewaysInput() * DriveConstants.MaxSpeed;
    }

    /**
    * Returns a double containing our Rotational input
    */
    public default double omegaRadiansPerSecond(){
        return omegaInput() * DriveConstants.MaxAngularRate;
    }

    
    public default double checkDeadzone(double input) {
        if (DriveConstants.JoystickDeadzone > input && input > -DriveConstants.JoystickDeadzone) {
          return 0;
        } else {
          return input;
        }
      }
    
    /**
    * Returns a ChassisSpeeds of our inputs wiht deadzone multiplied by our Max speed inputs
    */
    public default ChassisSpeeds inputSpeeds(){
        return new ChassisSpeeds(
            ((ForwardMetersPerSecond())),
            ((SidewaysMetersPerSecond())),
            ((omegaRadiansPerSecond())));
    }    
        
    /**
    * Returns a Field Relitive ChassisSpeeds
    */
    public default ChassisSpeeds fieldSpeeds(SwerveDriveSubsystem m_SwerveDriveSubsystem){
    return ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeeds(),
        new Rotation2d(m_SwerveDriveSubsystem.getPigeon2().getRotation2d().getRadians())
            // .rotateBy(new Rotation2d(-fieldRelativeOffset))
            );
    }     

} 