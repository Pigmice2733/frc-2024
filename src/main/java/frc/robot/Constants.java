// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pigmice.frc.lib.drivetrain.swerve.SwerveConfig;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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

    public static final class CANConfig {
        public static final int FRONT_LEFT_DRIVE = 11; // done
        public static final int FRONT_LEFT_STEER = 10;// done
        public static final int FRONT_RIGHT_DRIVE = 13;// done
        public static final int FRONT_RIGHT_STEER = 12;// done
        public static final int BACK_LEFT_DRIVE = 16;// done
        public static final int BACK_LEFT_STEER = 17;// done
        public static final int BACK_RIGHT_DRIVE = 14;// done
        public static final int BACK_RIGHT_STEER = 15;// done

        public static final int FRONT_LEFT_ABS_ENCODER = 20;// done
        public static final int FRONT_RIGHT_ABS_ENCODER = 24;// done
        public static final int BACK_LEFT_ABS_ENCODER = 22;// done
        public static final int BACK_RIGHT_ABS_ENCODER = 26;// done

        public static final int ARM_ROTATION = 30;
        public static final int ARM_ENCODER = 31;

        public static final int CLIMBER_EXTENSION = 40;

        public static final int INTAKE_WHEELS = 50;
        public static final int INTAKE_PIVOT = 51;
        public static final int INTAKE_ENCODER = 52;

        public static final int SHOOTER_ROTATION = 60;

        public static final int WRIST_ROTATION = 70;
        public static final int WRIST_ENCODER = 71;

        public static final int INDEXER_ROTATION = 80;
    }

    public static final class DIOConfig {
        public static final int ARM_LIMIT_SWITCH = 0;
        public static final int INTAKE_LIMIT_SWITCH = 1;
        public static final int WRIST_LIMIT_SWITCH = 2;
        public static final int INTAKE_BEAM_BREAK = 3;
        public static final int INDEXER_BEAM_BREAK = 4;
    }

    public final static class DrivetrainConfig {
        public static final double MAX_DRIVE_SPEED = 4.5; // max meters / second
        public static final double MAX_TURN_SPEED = 5; // max radians / second
        public static final double SLOWMODE_MULTIPLIER = 0.5;

        // distance from the center of one wheel to another
        public static final double TRACK_WIDTH_METERS = 0.5842;

        private final static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH_METERS / 2,
                        TRACK_WIDTH_METERS / 2), // Front left
                new Translation2d(TRACK_WIDTH_METERS / 2,
                        -TRACK_WIDTH_METERS / 2), // Front right
                new Translation2d(-TRACK_WIDTH_METERS / 2,
                        TRACK_WIDTH_METERS / 2), // Back left
                new Translation2d(-TRACK_WIDTH_METERS / 2,
                        -TRACK_WIDTH_METERS / 2) // Back right
        );

        // Constants found in Sysid (volts)
        private static final SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(
                0.35493, 2.3014, 0.12872);

        // From what I have seen, it is common to only use a P value in path following
        private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2); // 3, 2.5
        private static final PIDController PATH_DRIVE_PID = new PIDController(0.3, 0, 0);
        private static final PIDController PATH_TURN_PID = new PIDController(0.31, 0, 0);

        // Offset from chassis center that the robot will rotate about
        private static final Translation2d ROTATION_CENTER_OFFSET = new Translation2d(0, 0);

        private static final MkSwerveModuleBuilder FRONT_LEFT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Front Left", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.FRONT_LEFT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.FRONT_LEFT_STEER)
                .withSteerEncoderPort(CANConfig.FRONT_LEFT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(73));

        private static final MkSwerveModuleBuilder FRONT_RIGHT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Front Right", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(1, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.FRONT_RIGHT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.FRONT_RIGHT_STEER)
                .withSteerEncoderPort(CANConfig.FRONT_RIGHT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(-99));

        private static final MkSwerveModuleBuilder BACK_LEFT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Back Left", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.BACK_LEFT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.BACK_LEFT_STEER)
                .withSteerEncoderPort(CANConfig.BACK_LEFT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(219));

        private static final MkSwerveModuleBuilder BACK_RIGHT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(SWERVE_TAB
                        .getLayout("Back Right", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(3, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.BACK_RIGHT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.BACK_RIGHT_STEER)
                .withSteerEncoderPort(CANConfig.BACK_RIGHT_ABS_ENCODER)
                .withSteerOffset(Math.toRadians(-285));

        public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig(
                FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE, BACK_LEFT_MODULE,
                BACK_RIGHT_MODULE,
                PATH_CONSTRAINTS, PATH_DRIVE_PID, PATH_TURN_PID,
                MAX_DRIVE_SPEED, MAX_TURN_SPEED,
                SLOWMODE_MULTIPLIER, KINEMATICS, DRIVE_FEED_FORWARD, SWERVE_TAB,
                ROTATION_CENTER_OFFSET);
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
            SPEAKER(0);

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
            SPEAKER(60);

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
            DOWN(30);

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
            // TODO
            public final static Pose2d HUMAN_PLAYER_PICKUP = new Pose2d(0, 0, new Rotation2d());
            public final static Pose2d AMP_SCORING = new Pose2d(0, 0, new Rotation2d());
            public final static Pose2d SPEAKER_SCORING = new Pose2d(0, 0, new Rotation2d());
            public final static Pose2d CLIMBING = new Pose2d(0, 0, new Rotation2d());
            public final static Pose2d CENTRAL_RING_SEARCH = new Pose2d(0, 0, new Rotation2d());
        }
    }

    public static class ControlBindings {
        public final static int SCORE_AMP_BUTTON = Button.kX.value;
        public final static int SCORE_SPEAKER_BUTTON = Button.kY.value;
        public final static int INTAKE_GROUND_BUTTON = Button.kB.value;
        public final static int CLIMB_BUTTON = Button.kA.value;

    }
}
