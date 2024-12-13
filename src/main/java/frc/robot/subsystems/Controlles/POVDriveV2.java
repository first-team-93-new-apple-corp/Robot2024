package frc.robot.subsystems.Controlles;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class POVDriveV2 implements ControllerSchemeIO {

    private CommandJoystick LeftStick;
    private CommandJoystick RightStick;
    private Supplier<Double> robotAngle;
    private double Angle;
  /**
   * An implementation of  {@link #the (ControllerSchemeIO)}
   * <p> Uses left stick to generate center of rotation with FeildRel
   */
    public POVDriveV2(int LeftPort, int RightPort, Supplier<Double> robotAngle) {
        LeftStick = new CommandJoystick(LeftPort);
        RightStick = new CommandJoystick(RightPort);
        this.robotAngle = robotAngle;
        LeftStick.button(2).onTrue((new InstantCommand(() -> Angle = this.robotAngle.get())));
    }

    @Override
    public double InputLeft() {
        if (LeftStick.button(2).getAsBoolean()) {
            return 0;
        } else {
            return -LeftStick.getY();
        }
    }

    @Override
    public double InputUp() {
        if (LeftStick.button(2).getAsBoolean()) {
            return 0;
        } else {
            return -LeftStick.getX();
        }
    }

    @Override
    public double InputTheta() {
        return -RightStick.getX();
    }

    @Override
    public Translation2d POV() {
        if (LeftStick.button(2).getAsBoolean()) {
            return new Translation2d(-LeftStick.getY() ,-LeftStick.getX()).rotateAround(new Translation2d(0,0), Rotation2d.fromDegrees(180-Angle));
        } else {
            switch (LeftStick.getHID().getPOV()) {
                case 0:
                    return POVs[1];
                case 45:
                    return POVs[2];
                case 90:
                    return POVs[3];
                case 135:
                    return POVs[4];
                case 180:
                    return POVs[5];
                case 225:
                    return POVs[6];
                case 270:
                    return POVs[7];
                case 315:
                    return POVs[8];
                default:
                    return POVs[0];
            }
        }
    }

    @Override
    public Trigger Seed() {
        return LeftStick.button(12);
    }

    @Override
    public Trigger Brake(){
        return RightStick.trigger();
    }

}