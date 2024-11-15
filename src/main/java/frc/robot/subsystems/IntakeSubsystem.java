
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    PWMVictorSPX leftMotor;
    PWMVictorSPX rightMotor;
    public IntakeCommand Commands = new IntakeCommand();
    public IntakeSubsystem() {
        leftMotor = new PWMVictorSPX(0);
        rightMotor =  new PWMVictorSPX(1);
    }
    
    private void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(-speed);
    }
        

    public class IntakeCommand {
        public Command intake() {
            return runOnce(() -> {
                setSpeed(1);
            });
        }
        public Command outake() {
            return runOnce(() -> {
                setSpeed(-0.4);
            });
        }
    }
}
