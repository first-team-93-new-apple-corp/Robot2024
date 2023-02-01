package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


// uses a neo 550 for actuating
// uses a neo 550 for running the intaking wheels 
public class GrabberSubsystem extends SubsystemBase implements ArmInterface{

CANSparkMax Grabber;

    public GrabberSubsystem(){
  Grabber = new CANSparkMax(0, MotorType.kBrushless);
    }
    public void directMotorCommand(double speed){ //for manually opening and closing the grabber, that way position is a constant in this naming convention. the wheels will be dealt with elsewhere.
      Grabber.set(speed);
    }
    public void toSetpoint(double setpoint){ //for actuation
    
    }
    public void stopMotors(){
        
    }
  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
