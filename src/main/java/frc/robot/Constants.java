// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final ShuffleboardTab DRIVER_TAB = Shuffleboard.getTab("Driver");
    public static final ShuffleboardTab SWERVE_TAB = Shuffleboard.getTab("Drivetrain");
    public static final ShuffleboardTab ARM_TAB = Shuffleboard.getTab("Arm");
    public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");
    public static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Intake");
    public static final ShuffleboardTab INDEXER_TAB = Shuffleboard.getTab("Indexer");
    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Shooter");
    public static final ShuffleboardTab WRIST_TAB = Shuffleboard.getTab("Wrist");
    public static final ShuffleboardTab VISION_TAB = Shuffleboard.getTab("Vision");

    public static final double AXIS_THRESHOLD = 0.25;

    public static final double ROBOT_WIDTH = Units.inchesToMeters(22.953); // in meters (TODO)

    public static final class CANConfig {
        public static final int LEFT_ARM = 31;
        public static final int RIGHT_ARM = 30;
        public static final int ARM_ENCODER = 50;

        public static final int LEFT_CLIMB = 38;
        public static final int RIGHT_CLIMB = 39;

        public static final int INTAKE_WHEELS = 37;
        public static final int INTAKE_PIVOT = 36;
        public static final int INTAKE_ENCODER = 51;

        public static final int LEFT_SHOOTER = 34;
        public static final int RIGHT_SHOOTER = 35;

        public static final int WRIST_ROTATION = 32;
        public static final int WRIST_ENCODER = 52;

        public static final int LEFT_INDEXER = 33;
        public static final int RIGHT_INDEXER = 40;
    }

    public static final class DIOConfig {
        public static final int ARM_LIMIT_SWITCH = 0;
        public static final int INTAKE_LIMIT_SWITCH = 1;
        public static final int WRIST_LIMIT_SWITCH = 2;
        public static final int INTAKE_BEAM_BREAK = 3;
        public static final int INDEXER_BEAM_BREAK = 4;
    }

    public final static class DrivetrainConfig {
        public static final double MAX_DRIVE_SPEED = 2; // max meters / second
        public static final double MAX_TURN_SPEED = 2; // max radians / second
        public static final double SLOWMODE_MULTIPLIER = 0.5;

        // distance from the center of one wheel to another
        public static final double TRACK_WIDTH_METERS = 0.5842;

        public static final double DRIVE_BASE_RADIUS = 0.41;// Drive base radius in meters. Distance from robot center
                                                            // to furthest module.

        // TODO: sort out speeds and sync with path config
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                1, 1, 1, 1);

        // From what I have seen, it is common to only use a P value in path following
        private static final PIDConstants PATH_DRIVE_PID = new PIDConstants(5.0, 0, 0);
        private static final PIDConstants PATH_TURN_PID = new PIDConstants(5.0, 0, 0);

        public static final double MAX_PATH_DRIVE_SPEED = 1;
        // public static final double MAX_PATH_TURN_SPEED = 1;

        // Offset from chassis center that the robot will rotate about
        // private static final Translation2d ROTATION_CENTER_OFFSET = new
        // Translation2d(0, 0);

        public static final HolonomicPathFollowerConfig PATH_CONFIG = new HolonomicPathFollowerConfig(
                PATH_DRIVE_PID, PATH_TURN_PID, MAX_PATH_DRIVE_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig());
    }

    public final static class ArmConfig {
        public static final double P = 0;
        public static final double i = 0;
        public static final double D = 0;

        public static final double MAX_ACCELERATION = 0;
        public static final double MAX_VELOCITY = 0;

        public static final double MOTOR_POSITION_CONVERSION = 1;

        public static final double POSITION_TOLERANCE = 1;

        public static enum ArmState {
            STOW(90),
            AMP(45),
            SPEAKER(0),
            SOURCE(30);

            private double position;

            ArmState(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public final static class ClimberConfig {
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final double MAX_ACCELERATION = 0;
        public static final double MAX_VELOCITY = 0;

        public static final double MOTOR_POSITION_CONVERSION = 1;

        public static final double POSITION_TOLERANCE = 1;

        public static enum ClimberState {
            UP(45),
            DOWN(0);

            private double position;

            ClimberState(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public final static class WristConfig {
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final double MAX_ACCELERATION = 0;
        public static final double MAX_VELOCITY = 0;

        public static final double MOTOR_POSITION_CONVERSION = 1;

        public static final double POSITION_TOLERANCE = 1;

        public static enum WristState {
            STOW(0),
            AMP(30),
            SPEAKER(60),
            SOURCE(45);

            private double position;

            WristState(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public final static class IntakeConfig {
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final double MAX_ACCELERATION = 0;
        public static final double MAX_VELOCITY = 0;

        public static final double MOTOR_POSITION_CONVERSION = 1;

        public static final double POSITION_TOLERANCE = 1;

        public static final double WHEELS_SPEED = 0.3;

        public static enum IntakeState {
            STOW(0),
            DOWN(30),
            UP(10);

            private double position;

            IntakeState(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public final static class ShooterConfig {
        public static final double DEFAULT_SPEED = 0.3;
        public static final double BACKWARD_SPEED = -0.3;
    }

    public final static class IndexerConfig {
        public static final double DEFAULT_SPEED = 0.3;
        public static final double BACKWARD_SPEED = -0.3;
    }

    public final static class VisionConfig {
        public final static String CAM_NAME = "";
    }

    /** Details for auto such as timings and speeds. All times in seconds. */
    public static class AutoConfig {
        public final static double INTAKE_MOVE_TIME = 3;
        public final static double INTAKE_FEED_TIME = 1;
        public final static double SHOOTER_SPINUP_TIME = 3;
        public final static double CLIMB_DRIVE_TIME = 1;
        public final static double CLIMB_DRIVE_SPEED = 0.5;

        public static class Locations {
            // Start of the "lineup" path for these positions
            public final static Pose2d AMP_LINEUP = new Pose2d(1.82, 6.29, Rotation2d.fromDegrees(90));
            public final static Pose2d SPEAKER_LINEUP = new Pose2d(2.36, 5.56, Rotation2d.fromDegrees(180));
            public final static Pose2d SOURCE_LINEUP = new Pose2d(1.64, 1.83, Rotation2d.fromDegrees(-120));
            public final static Pose2d CLIMBING_LINEUP = new Pose2d(7.37, 4.03, Rotation2d.fromDegrees(180));

            // TODO: do we want lineup commands for all 3 climbing spots?
            // public final static Pose2d CLIMBING_RIGHT = new Pose2d(0, 0, new
            // Rotation2d());
            // public final static Pose2d CLIMBING_BACK = new Pose2d(0, 0, new
            // Rotation2d());

            public final static Pose2d CENTRAL_RING_SEARCH = new Pose2d(10.2, 4, new Rotation2d(0));
        }
    }

    public static class ControlBindings {
        // TODO decide these with strategy / drive team
        public final static int SCORE_AMP_BUTTON = Button.kX.value;
        public final static int SCORE_SPEAKER_BUTTON = Button.kY.value;
        public final static int INTAKE_GROUND_BUTTON = Button.kB.value;
        public final static int INTAKE_SOURCE_BUTTON = Button.kB.value;
        public final static int CLIMB_BUTTON = Button.kA.value;
        public final static int STOW_BUTTON = Button.kB.value;
    }
}
