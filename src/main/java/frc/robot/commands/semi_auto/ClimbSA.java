// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig.Locations;

public class ClimbSA extends SequentialCommandGroup {
    /** Drives to the chain and climbs */
    public ClimbSA(SwerveDrivetrain drivetrain, Pathfinder pathfinder) {
        addCommands(
                // TODO: implementation
                // Pathfind to the chain and stop in front of it
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.CLIMBING)
        // Climb
        );

        addRequirements();
    }
}
