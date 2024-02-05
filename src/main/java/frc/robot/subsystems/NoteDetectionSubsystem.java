package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;

public class NoteDetectionSubsystem extends SubsystemBase {
    TimeOfFlight AboveKicker = new TimeOfFlight(Constants.PWF.After_Kick);
    TimeOfFlight BelowKicker = new TimeOfFlight(Constants.PWF.Before_Kick);
    TimeOfFlight FrontBumber = new TimeOfFlight(Constants.PWF.FrontBumber);
    TimeOfFlight BackBumber = new TimeOfFlight(Constants.PWF.BackBumper);
    double BelowKickerVal = 150; // metric dist. to note
    double AboveKickerVal = 150; // metric dist. to note
    double FrontBumperVal = 150; // metric dist. to note
    double BackBumperVal = 150; // metric dist. to note
    boolean NoteInKicker = false;

    // Command in intake command
    public void NoteDectectionConstants() {
        if (ifBelowKicker() && ifAboveKicker()) {
            NoteInKicker = true;
        } else {

            NoteInKicker = false;
        }
        ifBelowKicker();
        ifAboveKicker();
        SmartDashboard.putNumber("Below Kicker Dist (Testing)", BelowKicker.getRange());
        SmartDashboard.putNumber("Above Kicker Dist (Testing)", AboveKicker.getRange());
        SmartDashboard.putBoolean("Note In Below Kicker", ifBelowKicker());
        SmartDashboard.putBoolean("Note In Above Kicker", ifAboveKicker());
        SmartDashboard.putBoolean("UnderBackBumper", ifFrontBumper());
        SmartDashboard.putBoolean("UnderFrontBumper", ifBackBumper());
        SmartDashboard.putBoolean("Note in Kicker", NoteInKicker);
    }

    public boolean ifBelowKicker() {
        if (BelowKicker.getRange() <= BelowKickerVal) {
            return true;
        } else {
            return false;
        }
    }

    public boolean ifAboveKicker() {
        if (AboveKicker.getRange() <= AboveKickerVal) {
            return true;
        } else {
            return false;
        }
    }

    public boolean ifFrontBumper() {
        if (FrontBumber.getRange() <= FrontBumperVal) {
            return true;
        } else {
            return false;
        }
    }

    public boolean ifBackBumper() {
        if (BackBumber.getRange() <= BackBumperVal) {
            return true;
        } else {
            return false;
        }
    }

}
