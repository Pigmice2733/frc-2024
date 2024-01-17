// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig.Locations;

public class ScoreAmpSA extends SequentialCommandGroup {
    /** Drives to the amp then scores */
    public ScoreAmpSA(SwerveDrivetrain drivetrain, Pathfinder pathfinder) {
        addCommands(
                // TODO: implementation
                // Pathfind to a point right in front of the amp
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.AMP_SCORING)
        // Score on the amp
        );

        addRequirements();
    }
}
