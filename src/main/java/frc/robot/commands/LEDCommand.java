// package frc.robot.commands;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.LEDSubsystem;

// public class LEDCommand extends Command {
//     LEDSubsystem LEDSubsystem;
//     XboxController op;

//     public LEDCommand(XboxController op) {
//         this.op = op;
//         LEDSubsystem = new LEDSubsystem();
//     }

//     @Override
//     public void execute() {
//         if (op.getRawAxis(2) != 0.5) {
//             LEDSubsystem.noteInBot();
//         } else {
//             return;
//         }
//     }

//     @Override
//     public void initialize() {
//         LEDSubsystem.Startup();
//     }
// }