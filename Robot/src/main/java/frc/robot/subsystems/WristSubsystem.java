package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//uses a neo for actuation

public class WristSubsystem extends SubsystemBase implements GenericMotorSubsystem{
    CANSparkMax WristMotor;
    RelativeEncoder WristEncoder;
    SparkMaxPIDController WristPID;
    CANCoder wristCanCoder;


    public WristSubsystem() {
        WristMotor = new CANSparkMax(Constants.CanID_Rev.WristMotor, MotorType.kBrushless);

        WristEncoder = WristMotor.getEncoder();
        wristCanCoder = new CANCoder(Constants.CanID_CTRE.WristCancoder); 
        WristPID = WristMotor.getPIDController();


    }

    public void toSetpoint(double setpointDegrees) {

        WristPID.setReference(setpointDegrees, ControlType.kSmartMotion);
    }

    public void directMotorCommand(double speed) {
        WristMotor.set(speed); 

    }
    public void stopMotors() {
        WristMotor.set(0); 
    }

    @Override
    public void periodic() {

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
