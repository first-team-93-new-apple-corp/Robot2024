
/*Nolens testing stuff for fun dont touch */



  package frc.robot.subsystems;

  import edu.wpi.first.math.geometry.Rotation2d;
  import edu.wpi.first.math.kinematics.SwerveModuleState;

  public class CTREModuleState {

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
      double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
      System.out.println(targetAngle);
      double targetSpeed = desiredState.speedMetersPerSecond;
      double delta = targetAngle - currentAngle.getDegrees();
      if (Math.abs(delta) > 90){
          targetSpeed = -targetSpeed;
          targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
      }        
      return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
       * @param scopeReference Current Angle
       * @param newAngle Target Angle
       * @return Closest angle within scope
       */
      private static double placeInAppropriate0To360Scope(double CurrentAngle, double newAngle) {
        double lowerBound = 0;
        double upperBound = 0;
        double ModulusCurrentAngle = CurrentAngle % 360;
        System.out.println(CurrentAngle);
        if (ModulusCurrentAngle >= 0) {
            lowerBound = CurrentAngle - ModulusCurrentAngle;
            upperBound = CurrentAngle + (360 - ModulusCurrentAngle);
        } else {
            upperBound = CurrentAngle - ModulusCurrentAngle;
            lowerBound = CurrentAngle - (360 + ModulusCurrentAngle);
        }
        System.out.println(upperBound);
        System.out.println(lowerBound);
        while (newAngle < lowerBound) {
            newAngle += 360;
            System.out.println("test");
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
            System.out.println(newAngle);

        }
        if (newAngle - CurrentAngle > 180) {
            newAngle -= 360;
        } else if (newAngle - CurrentAngle < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
  }