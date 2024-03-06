// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.vision.Vision;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivetrain extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private final Field2d fieldWidget;
    private final Vision vision;

    public Drivetrain(Vision vision) {
        this.vision = vision;

        try {
            swerveDrive = new SwerveParser(
                    new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(DrivetrainConfig.MAX_DRIVE_SPEED);
        } catch (IOException e) {
            e.printStackTrace();
        }

        AutoBuilder.configureHolonomic(
                () -> swerveDrive.getPose(), // Robot pose supplier
                swerveDrive::resetOdometry, // Method to reset odometry (will be
                                            // called if your auto has
                                            // a starting
                                            // pose)
                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST
                                               // BE ROBOT RELATIVE
                (val) -> {
                    swerveDrive.drive(new ChassisSpeeds(val.vxMetersPerSecond, val.vyMetersPerSecond,
                            val.omegaRadiansPerSecond));
                }, // Method that will drive the robot given
                   // ROBOT RELATIVE
                   // ChassisSpeeds
                DrivetrainConfig.PATH_FOLLOWER_CONFIG,
                () -> {
                    // Boolean supplier that controls when the path will be
                    // mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of
                    // the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // TODO: test this at some point (very important for actual
                    // matches)

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    // return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    // return false;

                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        // Initialize the field that shows on
        fieldWidget = new Field2d();
        Constants.SWERVE_TAB.add("Field", fieldWidget).withSize(7, 4);

        ShuffleboardHelper.addOutput("X Position", Constants.SWERVE_TAB,
                () -> swerveDrive.getPose().getX());
        ShuffleboardHelper.addOutput("Y Position", Constants.SWERVE_TAB,
                () -> swerveDrive.getPose().getY());
        ShuffleboardHelper.addOutput("Rotation", Constants.SWERVE_TAB,
                () -> swerveDrive.getPose().getRotation().getDegrees());

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            fieldWidget.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            fieldWidget.getObject("path").setPoses(poses);
        });
    }

    @Override
    public void periodic() {
        fieldWidget.setRobotPose(swerveDrive.getPose());
        addVisionMeasurements();
    }

    /** Adds vision measurements to correct the odometry */
    private void addVisionMeasurements() {
        if (vision == null)
            return;

        Pose2d estimatedPose = vision.getEstimatedRobotPose();

        if (estimatedPose == null)
            return;

        swerveDrive.addVisionMeasurement(estimatedPose,
                Timer.getFPGATimestamp());

    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public boolean withinDistanceOfPathEnd(Translation2d point, double positionTolerance) {
        return swerveDrive.getPose().getTranslation().getDistance(point) < positionTolerance;
    }

    public boolean withinDistanceOfPoint(Translation2d point, double positionTolerance) {
        return swerveDrive.getPose().getTranslation().getDistance(point) < positionTolerance;
    }
}
