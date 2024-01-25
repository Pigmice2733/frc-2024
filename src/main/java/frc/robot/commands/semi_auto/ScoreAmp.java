// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.pathfinder.PathfindToPointSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.AutoConfig.Locations;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreAmp extends SequentialCommandGroup {
    /** Drives to the amp then scores */
    public ScoreAmp(SwerveDrivetrain drivetrain, Pathfinder pathfinder, Arm arm, Wrist wrist, Shooter shooter,
            Indexer indexer, NoteSensor noteSensor, Intake intake) {

        addCommands(
                // Pathfind to a point right in front of the amp
                new PathfindToPointSwerve(drivetrain, pathfinder, Locations.AMP_SCORING),
                // Move intake out of the way
                intake.goToState(IntakeState.UP),
                // Move arm and wrist to states
                Commands.parallel(wrist.goToState(WristState.AMP), arm.goToState(ArmState.AMP)),
                // Push the note out
                indexer.indexBackward(),
                // Wait until note is out
                noteSensor.waitForNoNote(),
                // Stop running the indexer
                indexer.stopIndexer());

        addRequirements(drivetrain, arm, wrist, shooter, indexer, noteSensor, intake);
    }
}
