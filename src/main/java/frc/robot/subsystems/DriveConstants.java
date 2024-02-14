package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
        public static final double yOffset = 0;
        public static final double MaxSpeed = 0.5; // 6 Meters per second or 19.6850394 freedom units per second
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation in radians (for some reason)
        public static final double JoystickDeadzone = 0.07; //Change back tot 0.05 for merge to Robot dev if merged
        public static final double Track_Width = Units.inchesToMeters(35); // right wheel to left wheel
        public static final double Wheel_Base = Units.inchesToMeters(35); // Front To Back
        public static final Translation2d Center = new Translation2d(yOffset, 0);
        public static Translation2d dCenter = new Translation2d(
                        yOffset,
                        0);
        public static final Translation2d Front = new Translation2d(
                        (Wheel_Base / 2) + yOffset,
                        0);
        public static final Translation2d Right = new Translation2d(
                        yOffset,
                        (-Track_Width / 2));
        public static final Translation2d Back = new Translation2d(
                        (-Wheel_Base / 2) + yOffset,
                        0);
        public static final Translation2d Left = new Translation2d(
                        yOffset,
                        (Track_Width / 2));
        public static final Translation2d Location_FL = new Translation2d(
                        (Wheel_Base / 2) + yOffset,
                        (Track_Width / 2));
        public static final Translation2d Location_FR = new Translation2d(
                        (Wheel_Base / 2) + yOffset,
                        (-Track_Width / 2));
        public static final Translation2d Location_BL = new Translation2d(
                        (-Wheel_Base / 2) + yOffset,
                        (Track_Width / 2));
        public static final Translation2d Location_BR = new Translation2d(
                        (-Wheel_Base / 2) + yOffset,
                        (-Track_Width / 2));

}
