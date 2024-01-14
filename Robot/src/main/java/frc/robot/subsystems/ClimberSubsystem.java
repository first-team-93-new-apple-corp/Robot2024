package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements IClimber{
    //Initations here later
    Joystick DummyJoystick = new Joystick(0);
    WPI_TalonFX climberMotor1 = new WPI_TalonFX(0);
    WPI_TalonFX climberMotor2 = new WPI_TalonFX(1);
    boolean motor1Touch = false;
    boolean motor2Touch = false;
    @Override
    public void raise() {
        if (DummyJoystick.getRawButton(0)) { // Button for raising climber
            System.out.println("Release brake");
            lower();
        } else {
            climberMotor1.setNeutralMode(NeutralMode.Brake);
            climberMotor2.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void lower() {
        while (motor1Touch == false && motor2Touch == false) {
            System.out.println("Retract Springs");
            if (climberMotor1.getSupplyCurrent() > 20) { // Change current amount if needed
                motor1Touch = true;
            }
            if (climberMotor2.getSupplyCurrent() > 20) { // Change current amount if needed
                motor2Touch = true;
            }
        }
        System.out.println("Retract springs all the way");
        System.out.println("Supply extra power");
    }

	@Override
	public SubsystemBase asSubsystem() {
		return this;
	}

}

