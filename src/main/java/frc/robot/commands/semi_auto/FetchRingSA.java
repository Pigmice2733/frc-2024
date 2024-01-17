// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig.Locations;
import frc.robot.commands.actions.intake.IntakeFromGround;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class FetchRingSA extends SequentialCommandGroup {
    /** Drives to the human player then picks up a ring */
    public FetchRingSA(SwerveDrivetrain drivetrain, Pathfinder pathfinder, Intake intake, Indexer indexer, Arm arm,
            Wrist wrist, NoteSensor noteSensor) {
        addCommands(
                // Pathfind to a point right in front of the human player
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.HUMAN_PLAYER_PICKUP),
                // Pick up the ring
                new IntakeFromGround(intake, indexer, arm, wrist, noteSensor));

        addRequirements(drivetrain, intake, indexer, arm, wrist);
    }
}
