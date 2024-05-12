package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve.DriveConstants;

public interface InputsIO {

    /**
    * Returns a object containing all our inputs
    */
    public Pose2d Inputs();

    /**
    * Returns a double containing our forward input
    */
    public default double ForwardInput(){
        return Inputs().getX();
    }

    /**
    * Returns a double containing our Sideways input
    */
    public default double SidewaysInput(){
        return Inputs().getY();
    }

    /**
    * Returns a double containing our Rotational input
    */
    public default double omegaInput(){
        return Inputs().getRotation().getRadians();
    }

    private Pose2d Speeds(){
        return Inputs().times(DriveConstants.MaxSpeed);
    }
    
    /**
    * Returns a double containing our forward Speed
    */
    public default double ForwardMetersPerSecond(){
        return Speeds().getX();
    }

    /**
    * Returns a double containing our Sideways input
    */
    public default double SidewaysMetersPerSecond(){
        return Speeds().getY();
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
            (checkDeadzone(ForwardMetersPerSecond())),
            (checkDeadzone(SidewaysMetersPerSecond())),
            (checkDeadzone(omegaRadiansPerSecond())));
    }     

} 