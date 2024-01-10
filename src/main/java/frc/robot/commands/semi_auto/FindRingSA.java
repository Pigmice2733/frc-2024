// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig.Locations;

public class FindRingSA extends SequentialCommandGroup {
    /** Searches for a ring on the floor and picks it up */
    public FindRingSA(SwerveDrivetrain drivetrain, Pathfinder pathfinder) {
        addCommands(
                // TODO: implementation
                // Search for a ring (maybe pathfind to a central location)
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.CENTRAL_RING_SEARCH)
        // Once a ring is found, drive in front of it
        // Intake from ground
        );

        addRequirements();
    }
}
