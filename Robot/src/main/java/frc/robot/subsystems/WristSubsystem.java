package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//uses a neo for actuation

public class WristSubsystem extends SubsystemBase {

    CANSparkMax WristMotor;
    RelativeEncoder WristEncoder;
    CANCoder wristCanCoder;
    public WristSubsystem(){
        WristMotor = new CANSparkMax(0, MotorType.kBrushless);
        WristEncoder = WristMotor.getEncoder();
        wristCanCoder = new CANCoder(0); //TODO Change ID
    }

    public void toSetpoint(double setpointDegrees){ 

    }

    public void directMotorCommand(){

    }
    

  @Override public void periodic() {

  }

  @Override
  public void simulationPeriodic() {}
}
