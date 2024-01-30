// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConfig;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drivetrain extends SubsystemBase {
        private SwerveDrive swerveDrive;

        public Drivetrain() {
                try {
                        swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                                        .createSwerveDrive(DrivetrainConfig.MAX_DRIVE_SPEED);
                } catch (IOException e) {
                        e.printStackTrace();
                }

                AutoBuilder.configureHolonomic(
                                swerveDrive::getPose, // Robot pose supplier
                                swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has
                                                            // a starting
                                                            // pose)
                                swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                swerveDrive::drive, // Method that will drive the robot given ROBOT RELATIVE
                                                    // ChassisSpeeds
                                DrivetrainConfig.PATH_CONFIG,
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        // TODO: test this at some point (very important for actual matches)

                                        // var alliance = DriverStation.getAlliance();
                                        // if (alliance.isPresent()) {
                                        // return alliance.get() == DriverStation.Alliance.Red;
                                        // }
                                        // return false;

                                        return false;
                                },
                                this // Reference to this subsystem to set requirements
                );
                ShuffleboardHelper.addOutput("FL angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[0].angle.getDegrees());

                ShuffleboardHelper.addOutput("FR angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[1].angle.getDegrees());

                ShuffleboardHelper.addOutput("BL angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[2].angle.getDegrees());

                ShuffleboardHelper.addOutput("BR angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[3].angle.getDegrees());

                ShuffleboardHelper.addOutput("FL target", Constants.SWERVE_TAB,
                                () -> SwerveDriveTelemetry.desiredStates[0]);

                ShuffleboardHelper.addOutput("FR target", Constants.SWERVE_TAB,
                                () -> SwerveDriveTelemetry.desiredStates[2]);

                ShuffleboardHelper.addOutput("BL target", Constants.SWERVE_TAB,
                                () -> SwerveDriveTelemetry.desiredStates[4]);

                ShuffleboardHelper.addOutput("BR target", Constants.SWERVE_TAB,
                                () -> SwerveDriveTelemetry.desiredStates[6]);

                ShuffleboardHelper.addOutput("FL angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[0].angle.getDegrees());

                ShuffleboardHelper.addOutput("FR angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[1].angle.getDegrees());

                ShuffleboardHelper.addOutput("BL angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[2].angle.getDegrees());

                ShuffleboardHelper.addOutput("BR angle", Constants.SWERVE_TAB,
                                () -> swerveDrive.getModulePositions()[3].angle.getDegrees());
        }

        public SwerveDrive getSwerveDrive() {
                return swerveDrive;
        }
}
