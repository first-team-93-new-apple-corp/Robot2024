package frc.robot.DriveInputs;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Driver {
    private SendableChooser<InputsIO> DriveChooser;

    private InputsIO[] InputOptions;

    public Driver(int JoystickDriver1Port, int JoystickDriver2Port, int XboxDriverPort) {
        InputOptions = new InputsIO[]{new TwoStickDriveLinear(JoystickDriver1Port, JoystickDriver2Port),
            new TwoStickExponentialDrive(JoystickDriver1Port, JoystickDriver2Port),
            new XboxDriveLinear(XboxDriverPort),
            new XboxDriveExponential(XboxDriverPort),
            new TwoStickAcellerationDrive(JoystickDriver1Port, JoystickDriver2Port),
            new XboxAcellerationDrive(XboxDriverPort)
        };
        DriveChooser = new SendableChooser<InputsIO>();
        for (InputsIO inputsIO : InputOptions) {
            DriveChooser.setDefaultOption(inputsIO.getClass().getName().substring(22), inputsIO);

        }
        SmartDashboard.putData("DriveChoose",DriveChooser);
    }
    public InputsIO getDriver() {

        return DriveChooser.getSelected();

    }
}
