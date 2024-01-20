package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.ShooterAndIntakeCmd;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;

public class ClimberSubsystem  extends SubsystemBase{
    TalonFX climberMotor1 = new TalonFX(5);
    TalonFX climberMotor2 = new TalonFX(5);
    PIDController climber1PID = new PIDController(0,0,0);
    PIDController climber2PID = new PIDController(0,0,0);
    boolean motor1Hit = false;
    boolean motor2Hit = false;
    double startSetpoint = 0;
    double extendedSetpoint = 1000;
    double currentToStop = 2;
    double loweringSpeed = 0.1;
    ElevatorFeedforward feedforward1 = new ElevatorFeedforward(0,0,0,0); // change values
    ElevatorFeedforward feedforward2 = new ElevatorFeedforward(0,0,0,0); // change values
    

    enum climberState {
        IDLE,RAISED,LOWERING,LOWERED;
    }

    climberState state = climberState.IDLE;
    
    public ClimberSubsystem() {
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);
        climberMotor1.setNeutralMode(NeutralModeValue.Brake);

    }
    public void raiseClimber() {
        state = climberState.RAISED;
    }
    public void lowerClimber() {
        state = climberState.LOWERING;       
    }
  
    
        
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber1 Encoder", climberMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber2 Encoder", climberMotor2.getPosition().getValueAsDouble());
        SmartDashboard.putString("Climber State", state.toString());
        if (state.equals(climberState.RAISED)) {
            climberMotor1.set(climber1PID.calculate(extendedSetpoint));
            climberMotor2.set(climber1PID.calculate(extendedSetpoint));
        } else if (state.equals(climberState.IDLE)) {
            climberMotor1.set(climber1PID.calculate(startSetpoint));
            climberMotor2.set(climber1PID.calculate(startSetpoint));
        } else if (state.equals(climberState.LOWERED)) {
            climberMotor1.set(feedforward1.calculate(climber1PID.calculate(startSetpoint)));
            climberMotor2.set(feedforward2.calculate(climber1PID.calculate(startSetpoint)));
        } else if (state.equals(climberState.LOWERING)) { 
            if (climberMotor1.getSupplyCurrent().getValue() > 2) {
                motor1Hit = true;
                climberMotor1.set(0);
                climberMotor1.setNeutralMode(NeutralModeValue.Brake);
            }
            if (climberMotor2.getSupplyCurrent().getValue() > 2) {
                motor2Hit = true;
                climberMotor2.set(0);
                climberMotor2.setNeutralMode(NeutralModeValue.Brake);
            } 
            if (!motor1Hit) {
                climberMotor1.set(loweringSpeed);
            }
            if (!motor2Hit) {
                climberMotor1.set(loweringSpeed);
            }
            if (motor1Hit && motor2Hit) {
                state = climberState.LOWERED;
            }
     
        } 
        
    }
}
