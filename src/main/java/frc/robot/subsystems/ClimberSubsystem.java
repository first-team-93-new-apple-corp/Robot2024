package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.ShooterAndIntakeCmd;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;

public class ClimberSubsystem  extends SubsystemBase{
    TalonFX climberMotor1 = new TalonFX(5);
    TalonFX climberMotor2 = new TalonFX(5);
    PIDController climber1PID = new PIDController(0,0,0);
    PIDController climber2PID = new PIDController(0,0,0);
    double extendedSetpoint = 1000;
    double startSetpoint = 0;
    
    public ClimberSubsystem() {
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);
    }
    public void raiseClimber() {
        while (climberMotor1.getPosition().getValue()!= extendedSetpoint && climberMotor2.getPosition().getValue() != extendedSetpoint) {
            climberMotor1.set(climber1PID.calculate(extendedSetpoint));
            climberMotor2.set(climber1PID.calculate(extendedSetpoint));
        }
        
    }
    public void lowerClimber() {
        while (bothHit() == false){
            if (climberMotor1.getSupplyCurrent().getValue() > 2) {
                climberMotor1.setNeutralMode(NeutralModeValue.Brake);
            } else {
                climberMotor1.set(0.1);
            }
            if (climberMotor2.getSupplyCurrent().getValue() > 2) {
                climberMotor2.setNeutralMode(NeutralModeValue.Brake);
            } else {
                climberMotor2.set(0.1);
            }
            
        }
        while (climberMotor1.getPosition().getValue()!= startSetpoint && climberMotor2.getPosition().getValue() != startSetpoint) {
            climberMotor1.set(climber1PID.calculate(startSetpoint));
            climberMotor1.set(climber2PID.calculate(startSetpoint));
        }
    }
    public boolean bothHit() {
        if (climberMotor2.getSupplyCurrent().getValue() > 2 && climberMotor1.getSupplyCurrent().getValue() > 2) {
            return true;
        } else {
            return false;
        }
    }
    public void ClimberConstants(){
        SmartDashboard.putNumber("Climber1 Encoder", climberMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber2 Encoder", climberMotor2.getPosition().getValueAsDouble());
    }
    
}
