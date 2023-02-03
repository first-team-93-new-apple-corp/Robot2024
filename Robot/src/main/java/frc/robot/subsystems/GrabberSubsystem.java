package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// uses a neo 550 for actuating
// uses a neo 550 for running the intaking wheels 
public class GrabberSubsystem extends SubsystemBase implements GenericMotorSubsystem {

  CANSparkMax GrabberMotor;
  double CurrentCurrentOutput; 
  final double MaxAllowedCurrent = 20; 

  public GrabberSubsystem() {
    GrabberMotor = new CANSparkMax(0, MotorType.kBrushless);
    GrabberMotor.setIdleMode(IdleMode.kBrake); 



    CurrentCurrentOutput = 0; 
    
    SmartDashboard.putNumber("Grabber Motor Amp", CurrentCurrentOutput); 
    SmartDashboard.putBoolean("Grabber Motor Locked", false); 

  }

  public void directMotorCommand(double speed) { 

    
    if(CurrentCurrentOutput > MaxAllowedCurrent){
      GrabberMotor.set(0); 
    }
    else{
      GrabberMotor.set(speed);

    }

    


  }

  public void toSetpoint(double setpoint) { // for actuation

  }

  public void stopMotors() {
    GrabberMotor.set(0);
    

  }

  @Override
  public void periodic() {

    CurrentCurrentOutput = GrabberMotor.getOutputCurrent(); 

    SmartDashboard.putNumber("Grabber Motor Amp", CurrentCurrentOutput); 


  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public boolean atSetpoint() {
    // TODO Auto-generated method stub
    return false;
  }
}
