package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//uses a neo for actuation

public class WristSubsystem extends SubsystemBase implements ArmInterface{
    CANSparkMax WristMotor;
    RelativeEncoder WristEncoder;
    SparkMaxPIDController WristPID;
    CANCoder wristCanCoder;

    public WristSubsystem() {
        WristMotor = new CANSparkMax(0, MotorType.kBrushless);
        WristEncoder = WristMotor.getEncoder();
        wristCanCoder = new CANCoder(0); // TODO Change ID
        WristPID = WristMotor.getPIDController();

    }

    public void toSetpoint(double setpointDegrees) {
        WristPID.setReference(setpointDegrees, CANSparkMax.ControlType.kSmartMotion);
    }

    public void directMotorCommand(double speed) {

    }
    public void stopMotors() {
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
    }

}
