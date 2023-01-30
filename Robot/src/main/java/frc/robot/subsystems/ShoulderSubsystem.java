package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase implements ArmInterface{


  WPI_TalonFX ShoulderMotor1;
  WPI_TalonFX ShoulderMotor2;
  WPI_TalonFX ShoulderMotor3;
  WPI_TalonFX ShoulderMotor4;
  TalonFXConfiguration ShoulderMotorConfig;
  MotorControllerGroup ShoulderMotors;
  CANCoder shoulderCanCoder;
  public ShoulderSubsystem() {
    ShoulderMotor1 = new WPI_TalonFX(0); //verify ids
    ShoulderMotor2 = new WPI_TalonFX(0);
    ShoulderMotor3 = new WPI_TalonFX(0);
    ShoulderMotor4 = new WPI_TalonFX(0);
    //Use Motor Controller group once all motors spin the same direction
    ShoulderMotors = new MotorControllerGroup(ShoulderMotor1, ShoulderMotor2,ShoulderMotor3, ShoulderMotor4);
    ShoulderMotorConfig = new TalonFXConfiguration(); 
    ShoulderMotorConfig.motionAcceleration = 1;
    ShoulderMotorConfig.motionCurveStrength= 1;
    ShoulderMotorConfig.motionCruiseVelocity = 1;
    ShoulderMotorConfig.motionProfileTrajectoryPeriod = 1; //TODO tune these values, does this add a pid on top? do we need to tune that as well? investigate motion magic


    shoulderCanCoder = new CANCoder(0); //verify ids


  }
  public void toSetpoint(double setpointDegrees){ //TODO parameter should specify units.
  DegreesToRotations(setpointDegrees); 
  }

  public void directMotorCommand(double speed){

  }

  public void stopMotors() {
  }

  public double DegreesToRotations(double degrees){
    return degrees * Constants.DegreesToTicksShoulder;
  }

  public double TicksToDegrees(double Ticks){
    return Ticks * 1/Constants.DegreesToTicksShoulder;
  }

  public double getDegrees(){
    return TicksToDegrees(ShoulderMotor1.getSelectedSensorPosition() + ShoulderMotor2.getSelectedSensorPosition() + ShoulderMotor3.getSelectedSensorPosition() + ShoulderMotor4.getSelectedSensorPosition())/4;
  }

  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
