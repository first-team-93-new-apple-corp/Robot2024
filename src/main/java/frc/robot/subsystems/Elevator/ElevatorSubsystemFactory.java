package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.Utils;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.IO.ElevatorIOReal;
import frc.robot.subsystems.Elevator.IO.ElevatorIOSim;

public class ElevatorSubsystemFactory {
    public static ElevatorSubsystem build(ElevatorConstants constants) {
        
        if (Utils.isSimulation()) {
            return new ElevatorSubsystem(new ElevatorIOSim(constants));
        } else {
            return new ElevatorSubsystem(new ElevatorIOReal(constants));
        }        
    }
}
