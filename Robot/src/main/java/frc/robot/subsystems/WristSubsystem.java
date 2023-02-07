package frc.robot.subsystems;

import org.ejml.dense.row.decomposition.eig.SwitchingEigenDecomposition_DDRM;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;

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
        wristCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        wristCanCoder.configSensorDirection(true);
        wristCanCoder.configMagnetOffset(Constants.Wrist.configMagnetOffset);
        WristMotor.setInverted(true);
        WristEncoder.setPosition(DegreesToRotations(getDegrees()));
        WristPID.setP(0.00035,0);
        WristPID.setI(0,0);
        WristPID.setD(0,0);
        WristPID.setOutputRange(-0.5, 0.5,0);
        WristPID.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        WristPID.setSmartMotionMaxAccel(3000, 0);
        WristPID.setSmartMotionMaxVelocity(4500, 0);
        WristPID.setSmartMotionAllowedClosedLoopError(0.2, 0);
        SmartDashboard.putNumber("Wrist P", 0);
        SmartDashboard.putNumber("Wrist I", 0);
        SmartDashboard.putNumber("Wrist D", 0);
        SmartDashboard.putNumber("Wrist Acceleration", 0);
        SmartDashboard.putNumber("Wrist Velocity", 0);
        SmartDashboard.putNumber("Wrist Motor Velocity", WristEncoder.getVelocity());
   
        

    }

    public void toSetpoint(double SetpointDegrees) {
        double Rotations = DegreesToRotations(SetpointDegrees);
        WristPID.setReference(Rotations, CANSparkMax.ControlType.kSmartMotion);
    }
    public double getDegrees(){
        return wristCanCoder.getAbsolutePosition();
    }
    
    public double RotationsToDegrees(double Rotations){
        return (Rotations*360/120);
    }
    public double DegreesToRotations(double Degrees){
        return (Degrees/360*120);
    }

    public void directMotorCommand(double speed) {
        WristMotor.set(speed); 

    }
    public void stopMotors() {
        WristMotor.set(0); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Degrees", getDegrees());
        SmartDashboard.putNumber("Wrist Encoder Rotations", WristEncoder.getPosition());
        SmartDashboard.putNumber("Wrist Motor Rotations To Degrees", RotationsToDegrees(WristEncoder.getPosition()));
        SmartDashboard.putNumber("Wrist Degrees To Motor Rotations", DegreesToRotations(getDegrees()));
        // WristPID.setP(SmartDashboard.getNumber("Wrist P", 0));
        // WristPID.setI(SmartDashboard.getNumber("Wrist I", 0));
        // WristPID.setD(SmartDashboard.getNumber("Wrist D", 0));
        // WristPID.setSmartMotionMaxAccel(SmartDashboard.getNumber("Wrist Acceleration", 0), 0);
        // WristPID.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Wrist Velocity", 0), 0);
        SmartDashboard.putNumber("Wrist Motor Velocity", WristEncoder.getVelocity());
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
