package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    Servo AmpServo;
    public Commands Commands;

    public ServoSubsystem() {
        AmpServo = new Servo(8);
        Commands = new Commands(this);
    }

    public void setServo(double Position) {
        AmpServo.set(Position);
    }
    

    public class Commands {
        ServoSubsystem subsystem;

        Commands(ServoSubsystem subsystem) {

        }

        public Command ServoUp() {
            return runOnce(() -> setServo(0.1));
        }

        public Command ServoDown() {
            return runOnce(() -> setServo(0.65));
        }
    }
}
