package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * Contains all the robot constants
 */
public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class PhotonVision {
        public static final String Camera_Name_Back = "Camera_Intake_Side";
        public static final String camera_Name_Front = "Camera_Battery_Side";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d April_Tag_Front_pos =
            new Transform3d(new Translation3d(Units.inchesToMeters(-11.5d), Units.inchesToMeters(-10.7d), Units.inchesToMeters(9.0d)), new Rotation3d(Math.toRadians(-33d), 0, 0/*Math.PI+3.125*/));
        public static final Transform3d April_Tag_Back_pos =
            new Transform3d(new Translation3d(Units.inchesToMeters(-12.5d), Units.inchesToMeters(5.5d), Units.inchesToMeters(35.3d)), new Rotation3d(Math.toRadians(65d), 0, Math.toRadians(180)/*3.125*/));
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout tagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    /**
     * Contains all the Constants used by Path Planner
     */
    public static final class PathPlannerConstants {
        public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(2.0, 0, 0) // Rotation PID
        );

        public static final RobotConfig robotConfig;

        static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException e) {
                throw new RuntimeException(e);
            } catch (ParseException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //intake constans 
    public static final int Sparky_1 = 9;
    public static final int intake_motor_id = 50;
    public static final int intake_motor_max_rotations = 10;
    public static final double intake_motor_speed = 0.5;
   // lift arm
   public static final int ARM_MOTOR = 51;

    //shooter constants 
    private static final int shooter_motor_id = 999999;
    private static final int shooter_motor_max_rotation = 10;
public static final int motor2= 12;
public static final int motor1= 10;


        

        
}

  