package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import com.pathplanner.lib.util.GeometryUtil;

public class Constants {
    public IntakeConstants Intake = new IntakeConstants();
    public ElevatorConstants Elevator = new ElevatorConstants();
    public ClimberConstants Climber = new ClimberConstants();
    public SwerveDriveConstants Drive = new SwerveDriveConstants();
    public ShooterConstants Shooter = new ShooterConstants();
    public VisionConstants Vision = new VisionConstants();
    
    public enum BotName {
        SIM,
        Tobor27,
        Tobor26,
    }

    public final class AprilTagPoseConstants {

        public static final Pose2d RedAmp = new Pose2d(14.6, 7.6, new Rotation2d(Math.PI/2));
        public static final Pose2d BlueAmp = GeometryUtil.flipFieldPose(RedAmp);
        
    }

    public Constants(String robotName) {
        if (robotName.equals("2024")) {
            Elevator = new ElevatorConstants(18);
            Intake = new IntakeConstants(25, 14, 15, 22,25);
            Climber = new ClimberConstants(19,20);
            Shooter = new ShooterConstants(16, 17, 2, 2);
            Drive = new SwerveDriveConstants(BotName.Tobor27);
            Vision = new VisionConstants("LimeLight");

        } else if (robotName.equals("2023")) {
            Drive = new SwerveDriveConstants(BotName.Tobor26);
        } else if (robotName.equals("SIM")) {
            Elevator = new ElevatorConstants(18);
            Intake = new IntakeConstants(25, 14, 15, 22,25);
            Climber = new ClimberConstants(19,20);
            Drive = new SwerveDriveConstants(BotName.SIM);
            Vision = new VisionConstants("Photon");
        }
    }
    public final class ShooterConstants {
        public final boolean ENABLED;
        public final double SpeakerShooterSpeed = 0.55;
        public final double MuzzleIntake = -0.30;
        public final double AmpShooterSpeed = 0.1;
        public final double KickerSpeed = 1;
        public final double DribbleSpeed = .25;
        public final int ShooterL;
        public final int ShooterR;
        public final int KickerL;
        public final int KickerR;
        /**
         * Default constructor sets disabled
         */
        public ShooterConstants() {
          ENABLED = false;
          ShooterL = -1;
          ShooterR = -1;
          KickerL = -1;
          KickerR = -1;
        }

        /**
         * Sets the Ids of our Intake Motors
         *
         * @param ShooterL The Id of the Left Shooter
         * @param ShooterR The Id of the Right Shooter
         * @param KickerL The Id of the Left Kicker
         * @param KickerR The ID of the Right Kicker
         */
        public ShooterConstants(int ShooterL, int ShooterR, int KickerL, int KickerR) {
            ENABLED = true;
            this.ShooterL = ShooterL;
            this.ShooterR = ShooterR;
            this.KickerL = KickerL;
            this.KickerR = KickerR;
        }
    }
    public final class SwerveDriveConstants {
        public BotName DriveBase;
        public SwerveDriveConstants(){
            DriveBase = BotName.SIM;
        }
        public SwerveDriveConstants(BotName bot){
            DriveBase = bot;
        }
    }

    public final class ClimberConstants {
        public final boolean ENABLED;
        public final int leftClimber;
        public final int rightClimber;
        public final int maxHeight = 155;
        
        /*
         * Default constructor sets disabled
         */
        public ClimberConstants() {
            ENABLED = false;
            leftClimber = -1;
            rightClimber = -1;
        }
        /**
         * Sets the Ids of our Elevator Motor
         *
         * @param leftClimberID The Id of the Left Climber Motor
         * @param rightClimberID The ID of the Right Climber Motor
         */
        public ClimberConstants(int leftClimberID, int rightClimberID) {
            ENABLED = true;
            this.leftClimber = leftClimberID;
            this.rightClimber = rightClimberID;
        }
    }
    public final class ElevatorConstants {
        public final boolean ENABLED;
        public final int ElevatorMotor;
        public final double setpoint = 0;
        public final double highSetpoint = -75;
        public final double lowSetpoint = -3;
        /**
         * Default constructor sets disabled
         */
        public ElevatorConstants() {
          ENABLED = false;
          ElevatorMotor = -1;
        }

        /**
         * Sets the Ids of our Elevator Motor
         *
         * @param ElevatorID The Id of the Elevator Motor
         */
        public ElevatorConstants(int ElevatorID) {
            ENABLED = true;
            this.ElevatorMotor = ElevatorID;
        }
    }
    public final class IntakeConstants {
        public final boolean ENABLED;
        public static final double IntakeSpeed = 1;
        public static final double PassoverSpeed = 0.5;
        public final int midTOF;
        public final int upperTOF;
        public final int Bump_Intake;
        public final int F_Intake;
        public final int B_Intake;
        /**
         * Default constructor sets disabled
         */
        public IntakeConstants() {
          ENABLED = false;
          midTOF = -1;
          upperTOF = -1;
          Bump_Intake = -1;
          F_Intake = -1;
          B_Intake = -1;    
        }

        /**
         * Sets the Ids of our Intake Motors
         *
         * @param BumperIntake The Id of the Bumber intake
         * @param frontIntake The Id of the Front intake
         * @param backIntake The Id of the Back intake
         * @param midTOF The ID of the middle Time of Flight
         * @param upperTOF The ID of the upper Time of Flight
         */
        public IntakeConstants(int BumperIntake, int frontIntake, int backIntake, int midTOF, int upperTOF) {
            ENABLED = true;
            this.midTOF = midTOF;
            this.upperTOF = upperTOF;
            Bump_Intake = BumperIntake;
            F_Intake = frontIntake;
            B_Intake = backIntake;
        }
      }
    
        
    public class VisionConstants{
        public final boolean SimEnabled = true;
        public final String CameraName;
        public final Pose3d RobotToCamera = new Pose3d(0.321, 0.244354, 0.548, new Rotation3d(0, 30, 0));
        public VisionConstants(){
            CameraName = "LimeLight";
        }
        public VisionConstants(String CameraName){
            this.CameraName = CameraName;
        }
    }

    public class REV {
        public static final int L_Kicker = 2;
        public static final int R_Kicker = 3;
    }

    public class CTRE {
        public class RIO {
            public static final int Bump_Intake = 25;
            public static final int F_Intake = 14;
            public static final int B_Intake = 15;
            
            public static final int L_Shoot = 16;
            public static final int R_Shoot = 17;

            public static final int Elevator = 18;

            public static final int L_Climber = 19;
            public static final int R_Climber = 20;
        }

        public class Canivore {
            public static final int FLDrive = 1;
            public static final int FRDrive = 2;
            public static final int BR_Drive = 3;
            public static final int BL_Drive = 4;

            public static final int FL_Steer = 5;
            public static final int FR_Steer = 6;
            public static final int BR_Steer = 7;
            public static final int BL_Steer = 8;

            public static final int FL_Cancoder = 10;
            public static final int FR_Cancoder = 11;
            public static final int BR_Cancoder = 12;
            public static final int BL_CanCoder = 13;
        }
    }

    public class Thrustmaster {
        public static final int Trigger = 1;
        public static final int Center_Button = 2;
        public static final int Left_Button = 3;
        public static final int Right_Button = 4;

        public class Left_Buttons {
            public static final int Top_Left = 11;
            public static final int Top_Middle = 12;
            public static final int Top_Right = 13;
            public static final int Bottom_Left = 16;
            public static final int Bottom_Middle = 15;
            public static final int Bottom_Right = 14;
        }

        public class Right_Buttons {
            public static final int Top_Left = 7;
            public static final int Top_Middle = 6;
            public static final int Top_Right = 5;
            public static final int Bottom_Left = 8;
            public static final int Bottom_Middle = 9;
            public static final int Bottom_Right = 10;
        }

        public class Axis {
            public static final int y = 1;
            public static final int x = 0;
            public static final int rotate = 2;
            public static final int slider = 3;
        }
    }

    public class F310_D {
        public static final int X = 1;
        public static final int A = 2;
        public static final int B = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int LeftTrigger = 7;
        public static final int RightTrigger = 8;
        public static final int Back = 9;
        public static final int Start = 10;
        public static final int LeftStick = 11;
        public static final int RightStick = 12;

        public class Axis {
            public static final int POV_Y = 0;
            public static final int POV_X = 1;
            public static final int Left_Stick_Y = 0;
            public static final int Left_Stick_X = 1;
            public static final int Right_Stick_Y = 3;
            public static final int Right_Stick_X = 2;
        }
    }

    public class F310_X {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int Back = 7;
        public static final int Start = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;

        public class Axis {
            public static final int POV_Y = 1;
            public static final int POV_X = 0;
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int Left_Stick_Y = 1;
            public static final int Left_Stick_X = 0;
            public static final int Right_Stick_Y = 5;
            public static final int Right_Stick_X = 4;
        }
    }
    public class xbox {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LeftShoulderButton = 5;
        public static final int RightShoulderButton = 6;
        public static final int Window = 7;
        public static final int Menu = 8;
        public static final int LeftPaddle = 9;
        public static final int RightPaddle = 10;

        public class Axis {
            public static final int LT = 2;
            public static final int RT = 3;
            public static final int Left_Stick_Y = 1;
            public static final int Left_Stick_X = 0;
            public static final int Right_Stick_Y = 5;
            public static final int Right_Stick_X = 4;
        }
    }
}
