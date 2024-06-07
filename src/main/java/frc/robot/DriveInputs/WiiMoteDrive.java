package frc.robot.DriveInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class WiiMoteDrive implements InputsIO{
    private final GenericHID Wiimote;
    private double speed;
    private double Acelleration = .005;
    /**
    * Creates an object containing the Wiimote that will return our input values
    * The drive is Mario Kart Style
    */
    public WiiMoteDrive(int WiimotePort){
        this.Wiimote = new GenericHID(WiimotePort);
    }
    private double Steer(){
        return -Wiimote.getRawAxis(0);
    }
    private double SquareWithSign(double value){
        return Math.copySign(value * value, value);
    }

    @Override
    public Pose2d Inputs() {
        if (Wiimote.getRawButton(2)){
            speed += Acelleration;
        } else if( Wiimote.getRawButton(1)){
            speed -= Acelleration;
        } else {
            speed = (speed > 0) ? (speed -= 0.05) : (speed += 0.05);
        }
        speed = (speed >1) ? (1) : (speed);
        speed = (speed <-1) ? (-1) : (speed);
        return new Pose2d(
            SquareWithSign(speed),
            0,
            Rotation2d.fromRadians(Steer()));
    }

    /**
    * Returns a Field Relative ChassisSpeeds
    */
    @Override
    public ChassisSpeeds fieldSpeeds(Rotation2d RobotAngle){
    return inputSpeeds();
    }
    @Override
    public Trigger brake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'brake'");
    }
    @Override
    public Trigger fieldRelButton() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'fieldRelButton'");
    }
    @Override
    public Trigger robotRelButtonke() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'robotRelButtonke'");
    }
    @Override
    public Trigger ampAlignButton() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'ampAlignButton'");
    } 
    
}
