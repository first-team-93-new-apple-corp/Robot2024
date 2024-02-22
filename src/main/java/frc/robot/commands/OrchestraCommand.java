package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

public class OrchestraCommand {
    Orchestra orchestra = new Orchestra();

    public OrchestraCommand() {
        // for (int i = 1; i != 8; i++) {
        //     orchestra.addInstrument(new TalonFX(i, "drivetrain"));
        // }
        // for (int i = 14; i != 18; i++) {
        //     orchestra.addInstrument(new TalonFX(i, "rio"));
        // }
        
        // orchestra.addInstrument(new TalonFX(19, "drivetrain"));
        // orchestra.addInstrument(new TalonFX(20, "drivetrain"));

        orchestra.loadMusic("output2.chrp");
    }

    public void play() {
        orchestra.play();
    }
}
