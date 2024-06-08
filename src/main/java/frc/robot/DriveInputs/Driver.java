package frc.robot.DriveInputs;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Driver {
    private enum DriveStates {
        TwoStickDriveLinear,
        TwoStickExponentialDrive,
        XboxDriveLinear,
        XboxDriveExponential,
        TwoStickAcellerationDrive,
        XboxAcellerationDrive,
        XboxTank,
        WiiMoteDrive,
        OneStickLinear,
        OneStickDriveExponential,
        CollisionDeteaction
    }

    private final SendableChooser<DriveStates> DriveChooser2;
    private InputsIO CurrentSelection = new TwoStickDriveLinear(0, 1);
    private final int JoystickDriver1Port;
    private final int JoystickDriver2Port;
    private final int XboxDriverPort;
    public Boolean upToDate = true;
    private Supplier<Pose2d> pose;

    /**
     * This is the constructor for our Driver, makes a SmartDashboard chooser of all the options and allows you to switch between them
     */
    public Driver(int JoystickDriver1Port, int JoystickDriver2Port, int XboxDriverPort, Supplier<Pose2d> pose) {
        this.XboxDriverPort = XboxDriverPort;
        this.JoystickDriver1Port = JoystickDriver1Port;
        this.JoystickDriver2Port = JoystickDriver2Port;
        this.pose = pose;
        DriveChooser2 = new SendableChooser<DriveStates>();

        for (DriveStates State : DriveStates.values()) {
            DriveChooser2.addOption(State.name(), State);
        }

        DriveChooser2.setDefaultOption(DriveStates.XboxDriveLinear.name(), DriveStates.XboxDriveLinear);
        SmartDashboard.putData("DriveChooser", DriveChooser2);
        DriveChooser2.onChange(this::DriveMaker);
    }

    private void DriveMaker(DriveStates State) {
        unBind();
        upToDate = false;
        switch (State) {
            case TwoStickAcellerationDrive:
                CurrentSelection = new TwoStickAcellerationDrive(JoystickDriver1Port, JoystickDriver2Port);
                break;
            case TwoStickDriveLinear:
                CurrentSelection = new TwoStickDriveLinear(JoystickDriver1Port, JoystickDriver2Port);
                break;
            case TwoStickExponentialDrive:
                CurrentSelection = new TwoStickExponentialDrive(JoystickDriver1Port, JoystickDriver2Port);
                break;
            case WiiMoteDrive:
                CurrentSelection = new WiiMoteDrive(XboxDriverPort);
                break;
            case XboxAcellerationDrive:
                CurrentSelection = new XboxAcellerationDrive(XboxDriverPort);
                break;
            case XboxDriveExponential:
                CurrentSelection = new XboxDriveExponential(XboxDriverPort);
                break;
            case XboxDriveLinear:
                CurrentSelection = new XboxDriveLinear(XboxDriverPort);
                break;
            case XboxTank:
                CurrentSelection = new XboxTank(XboxDriverPort);
                break;
            case OneStickDriveExponential:
                CurrentSelection = new OneStickDriveExponential(JoystickDriver1Port);
                break;
            case OneStickLinear:
                CurrentSelection = new OneStickLinear(JoystickDriver1Port);
                break;
            case CollisionDeteaction:
                CurrentSelection = new TwoStickDriveCollisions(JoystickDriver2Port, JoystickDriver1Port, pose);
                break;

        }
    }

    public InputsIO getDrive() {
        return CurrentSelection;
    }

    public Trigger getBrakeButton() {
        return CurrentSelection.brake();
    }

    public Trigger getFieldRelButton() {
        return CurrentSelection.fieldRelButton();
    }

    public Trigger getRobotRelButton() {
        return CurrentSelection.robotRelButtonke();
    }

    public Trigger getAmpAlignButton() {
        return CurrentSelection.ampAlignButton();
    }

    public void unBind() {
        CurrentSelection.unBind();
    }

    public void Updated() {
        upToDate = true;
    }
}
