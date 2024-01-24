package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;

public class NoteDetectionSubsystem extends SubsystemBase {
    static TimeOfFlight AboveKicker = new TimeOfFlight(0);
    static TimeOfFlight BelowKicker = new TimeOfFlight(1);
    static double BelowKickerVal = 10; // metric dist. to note
    static double AboveKickerVal = 10; // metric dist. to note
    static boolean ifNoteBelowKicker = false;
    static boolean ifNoteAboveKicker = false;
    static boolean NoteInKicker = false;

    // Command in intake command
    public static void NoteDectectionConstants() {
        if (ifBelow() && ifAbove()) {
            NoteInKicker = true;
        } else {
            
            NoteInKicker = false;
        }
        ifBelow();
        ifAbove();
        SmartDashboard.putNumber("Below Kicker Dist (Testing)", BelowKicker.getRange());
        SmartDashboard.putNumber("Above Kicker Dist (Testing)", AboveKicker.getRange());
        SmartDashboard.putBoolean("Note In Below Kicker", ifBelow());
        SmartDashboard.putBoolean("Note In Above Kicker", ifAbove());
        SmartDashboard.putBoolean("Note in Kicker", NoteInKicker);
    }

    public static boolean ifBelow() {
        if (BelowKicker.getRange() == BelowKickerVal) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean ifAbove() {
        if (AboveKicker.getRange() == AboveKickerVal) {
            return true;
        } else {
            return false;
        }
    }

}
