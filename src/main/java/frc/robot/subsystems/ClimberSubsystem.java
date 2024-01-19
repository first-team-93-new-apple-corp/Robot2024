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
    TalonFX climberMotor1 = new TalonFX(1);
    TalonFX climberMotor2 = new TalonFX(2);
    PIDController climber1PID = new PIDController(0,0,0);
    PIDController climber2PID = new PIDController(0,0,0);
    double extendedSetpoint = 1000;
    double climber1Setpoint = 0;
    double climber2Setpoint = 0;
    public ClimberSubsystem() {
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);
    }
    public void raiseClimber() {
        climberMotor1.set(climber1PID.calculate(extendedSetpoint));
        climberMotor2.set(climber1PID.calculate(extendedSetpoint));
    }
    public void lowerClimber() {
        // // while ()
    }
    // public boolean bothHit() {
        // // if (climberMotor1.getCur)
    //}
}
