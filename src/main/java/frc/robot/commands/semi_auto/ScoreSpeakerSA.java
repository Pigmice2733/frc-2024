// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig.Locations;
import frc.robot.commands.actions.shooter.FireIntoSpeaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreSpeakerSA extends SequentialCommandGroup {
    /** Drives to the speaker then scores */
    public ScoreSpeakerSA(SwerveDrivetrain drivetrain, Pathfinder pathfinder, Arm arm, Wrist wrist, Shooter shooter,
            Indexer indexer) {

        addCommands(
                // Pathfind to a point right in front of the amp
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.AMP_SCORING),
                // Score on the speaker
                new FireIntoSpeaker(arm, wrist, shooter, indexer));

        addRequirements(drivetrain, arm, wrist, shooter, indexer);
    }
}
