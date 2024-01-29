// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends Command {
    private final Drivetrain drivetrain;
    private final Supplier<Double> driveSpeedX, driveSpeedY, turnSpeed;
    private final Supplier<Translation2d> centerOffset;

    /** The default drive command for swerve, but with a center offset */
    public DriveWithJoysticks(
            Drivetrain drivetrain, Supplier<Double> driveSpeedX, Supplier<Double> driveSpeedY,
            Supplier<Double> turnSpeed, Supplier<Translation2d> centerOffset) {
        this.drivetrain = drivetrain;
        this.driveSpeedX = driveSpeedX;
        this.driveSpeedY = driveSpeedY;
        this.turnSpeed = turnSpeed;
        this.centerOffset = centerOffset;

        addRequirements(drivetrain);
    }

    /** The default drive command for swerve */
    public DriveWithJoysticks(
            Drivetrain drivetrain, Supplier<Double> driveSpeedX, Supplier<Double> driveSpeedY,
            Supplier<Double> turnSpeed) {
        this(drivetrain, driveSpeedX, driveSpeedY, turnSpeed, null);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (centerOffset == null) {
            drivetrain.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(driveSpeedY.get(), driveSpeedX.get(),
                    turnSpeed.get()));
        } else {
            drivetrain.getSwerveDrive().driveFieldOriented(new ChassisSpeeds(driveSpeedY.get(), driveSpeedX.get(),
                    turnSpeed.get()), centerOffset.get());
        }

    }
}