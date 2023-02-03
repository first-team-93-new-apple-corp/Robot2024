package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.OperatorInterfaceSub;
import frc.robot.subsystems.WristSubsystem;

public class OperatorSelectorCommand extends CommandBase {


    boolean forward;
    OperatorInterfaceSub op;

    JoystickButton left, right; 

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public OperatorSelectorCommand(JoystickButton left, JoystickButton right,  OperatorInterfaceSub op) {
        this.left = left;
        this.right = right; 
        this.op = op;
    }

    @Override
    public void initialize() {



    }

    int loop_counter = 0; 
    int max_loops = (int) Units.secondsToMilliseconds(0.25) / 20; 
    @Override
    public void execute() {
       if(left.getAsBoolean() && loop_counter >= max_loops ){
            op.changeState(false);
            loop_counter = 0; 
        }
        else if(right.getAsBoolean() && loop_counter >= max_loops){
            op.changeState(true);
            loop_counter = 0; 
        }
        else{
            loop_counter++; 
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
