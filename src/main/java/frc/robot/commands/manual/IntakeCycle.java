// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class IntakeCycle extends SequentialCommandGroup {
    /** Creates a new IntakeCycle. */
    public IntakeCycle(Intake intake, Indexer indexer, Arm arm, Wrist wrist, NoteSensor noteSensor) {
        addCommands(
                // Lower the intake
                intake.goToState(IntakeState.DOWN),

                // Start intake wheels and wait for a note in the intake
                Commands.parallel(intake.runWheelsForward(), noteSensor.waitForNoteInIntake()),

                // If the arm and wrist are not down, wait until they are
                Commands.either(
                        // Run if condition is true
                        Commands.none(),
                        // Run if conditions is false
                        Commands.parallel(intake.stopWheels(),
                                arm.waitForState(ArmState.STOW),
                                wrist.waitForState(WristState.STOW)),
                        // Condition
                        () -> arm.atState(ArmState.STOW) && wrist.atState(WristState.STOW)),

                // Start the intake and indexer wheels and wait for a note in the indexer
                Commands.parallel(intake.runWheelsForward(), indexer.indexForward(), noteSensor.waitForNoteInShooter()),

                // Stop the indexer and intake
                Commands.parallel(intake.stopWheels(), indexer.stopIndexer()));

        addRequirements(indexer, arm, wrist);
    }
}
