// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

// import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
// import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig.Locations;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.vision.Vision;

public class FindRing extends SequentialCommandGroup {
    /**
     * Searches for a ring on the floor and picks it up.
     */
    public FindRing(/* SwerveDrivetrain drivetrain, */Pathfinder pathfinder, Intake intake, Indexer indexer,
            NoteSensor noteSensor, Vision vision) {

        addCommands(
                // Search for a ring (maybe pathfind to a central location)
                // TODO: fix once the fixed drivetrain is merged in
                // Commands.parallel(
                // new PathfindToPointSwerve(drivetrain, pathfinder,
                // Locations.CENTRAL_RING_SEARCH),
                // vision.waitForRing()),
                // Once a ring is spotted, drive in front of it
                // TODO
                // Pick the ring up
                new IntakeFromGround(intake, indexer, noteSensor));

        addRequirements(/* TODO drivetrain, */intake, indexer, noteSensor);
    }
}
