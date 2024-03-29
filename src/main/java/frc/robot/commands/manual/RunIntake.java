// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class RunIntake extends SequentialCommandGroup {
    public RunIntake(Intake intake, Indexer indexer, Arm arm, Wrist wrist, Shooter shooter, NoteSensor noteSensor) {
        addCommands(
                // Stow kobra, keeping intake down
                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.STOW, false),

                // Start the intake + indexer
                Commands.parallel(intake.runWheelsForward(), indexer.indexForward()),

                // Wait for a note in the shooter
                noteSensor.waitForNoteInShooter(),

                // Stop all the wheels
                intake.stopWheels(), indexer.stopIndexer());
        addRequirements(indexer);
    }
}
