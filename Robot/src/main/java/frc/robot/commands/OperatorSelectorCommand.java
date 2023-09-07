package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.OperatorInterfaceSubsystem;

public class OperatorSelectorCommand extends CommandBase {

    boolean forward;
    OperatorInterfaceSubsystem op;

    JoystickButton left, right;
    int max_loops = (int) Units.secondsToMilliseconds(Constants.OperatorSettings.TimeBetweenSelectorPresses) / 20;
    int loopsBetweenLastPressed = 0;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public OperatorSelectorCommand(JoystickButton left, JoystickButton right, OperatorInterfaceSubsystem op) {
        this.left = left;
        this.right = right;
        this.op = op;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (left.getAsBoolean() && loopsBetweenLastPressed >= max_loops) {
            op.changeState(false);
            loopsBetweenLastPressed = 0;
        } else if (right.getAsBoolean() && loopsBetweenLastPressed >= max_loops) {
            op.changeState(true);
            loopsBetweenLastPressed = 0;
        } else {
            loopsBetweenLastPressed++;
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
