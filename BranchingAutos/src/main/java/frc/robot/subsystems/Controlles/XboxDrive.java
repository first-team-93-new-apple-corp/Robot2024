package frc.robot.subsystems.Controlles;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDrive implements ControllerIO {

    public CommandXboxController Xbox;

    public XboxDrive(int port) {
        Xbox = new CommandXboxController(port);
    }

    @Override
    public double InputLeft() {
        return -Xbox.getLeftY();
    }

    @Override
    public double InputUp() {
        return -Xbox.getLeftX();
    }

    @Override
    public double InputTheta() {
        return -Xbox.getRightX();
    }

    @Override
    public Translation2d POV() {
        return new Translation2d(0, 0);
    }

    @Override
    public Trigger Seed() {
        return Xbox.leftBumper();
    }

}
