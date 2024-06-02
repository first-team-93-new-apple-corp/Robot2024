package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve.DriveConstants;

public interface InputsIO {
    public EventLoop loop = new EventLoop();
    public default void unBind(){
        loop.clear();
    }
    public default void poll(){
        loop.poll();
    }
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
    * Returns a Field Relative ChassisSpeeds
    */
    public default ChassisSpeeds fieldSpeeds(Rotation2d Robotangle){
    return ChassisSpeeds.fromFieldRelativeSpeeds(inputSpeeds(), Robotangle);
    }    
    /**
     * Returns the Brake Trigger
     */
    public Trigger brake();
    public Trigger fieldRelButton();
    public Trigger robotRelButtonke();
    public Trigger ampAlignButton();    
} 