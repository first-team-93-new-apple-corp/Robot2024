package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    public Servo ampServo;
    public Commands Commands;
    public ServoSubsystem() {
        ampServo = new Servo(8);
        Commands = new Commands(this);
    }

    public void setServo(double position) {
        ampServo.set(position);
    }

    public class Commands {
        ServoSubsystem subsystem;

        public Commands(ServoSubsystem subsystem) {
            this.subsystem = subsystem;
        }

        public Command ServoUp() {
            return subsystem.runOnce(() -> {
                subsystem.setServo(0.1);
            });
        }
        public Command ServoDown() {
            return subsystem.runOnce(() -> {
                subsystem.setServo(0.65);
            });
        }
    }
}
