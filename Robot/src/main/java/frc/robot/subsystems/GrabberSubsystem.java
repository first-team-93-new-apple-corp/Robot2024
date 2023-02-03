package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// uses a neo 550 for actuating
// uses a neo 550 for running the intaking wheels 
public class GrabberSubsystem extends SubsystemBase implements GenericMotorSubsystem {

  CANSparkMax m_GrabberMotor;
  RelativeEncoder m_GrabberEncoder;
  double CurrentCurrentOutput;
  final double MaxAllowedCurrent = 20;

  public GrabberSubsystem() {
    m_GrabberMotor = new CANSparkMax(Constants.CanID_Rev.GrabberMotor, MotorType.kBrushless);
    m_GrabberMotor.setIdleMode(IdleMode.kBrake);

    m_GrabberEncoder = m_GrabberMotor.getEncoder();

    m_GrabberEncoder.setPosition(0);

    CurrentCurrentOutput = 0;

    SmartDashboard.putNumber("Grabber Motor Amp", CurrentCurrentOutput);
    SmartDashboard.putBoolean("Grabber Motor Locked", false);
    SmartDashboard.putNumber("Current Grabber Position", m_GrabberEncoder.getPosition());

  }

  public void directMotorCommand(double speed) {

    // if(CurrentCurrentOutput > MaxAllowedCurrent){
    // GrabberMotor.set(0);
    // }
    // else{
    m_GrabberMotor.set(speed);

    // }

  }

  public void toSetpoint(double setpoint) { // for actuation

  }

  public void stopMotors() {
    m_GrabberMotor.set(0);

  }

  @Override
  public void periodic() {

    CurrentCurrentOutput = m_GrabberMotor.getOutputCurrent();

    SmartDashboard.putNumber("Grabber Motor Amp", CurrentCurrentOutput);
    SmartDashboard.putNumber("Current Grabber Position", m_GrabberEncoder.getPosition());

  }


  @Override
  public boolean atSetpoint() {
    return false;
  }

  @Override
  public void simulationPeriodic() {
  }


}
