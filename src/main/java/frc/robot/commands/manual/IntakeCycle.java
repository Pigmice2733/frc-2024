// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class IntakeCycle extends SequentialCommandGroup {
    /** Creates a new IntakeCycle. */
    public IntakeCycle(Intake intake, Indexer indexer, Arm arm, Wrist wrist, NoteSensor noteSensor) {
        // Run intake until we have a note
        addCommands(Commands.race(intake.runWheelsForward(), noteSensor.waitForNoteInIntake()));

        // If the arm and wrist are not down, wait until they are
        addCommands(Commands.either(
                // Run if condition is true
                Commands.none(),
                // Run if conditions is false
                Commands.parallel(intake.stopWheels(),
                        Commands.parallel(
                                arm.waitForState(ArmState.STOW),
                                wrist.waitForState(WristState.STOW))),
                // Condition
                () -> arm.atState(ArmState.STOW) && wrist.atState(WristState.STOW)));

        // Run the intake and indexer until the note is in the indexer
        addCommands(Commands.race(
                Commands.parallel(intake.runWheelsForward(), indexer.indexForward()),
                noteSensor.waitForNoteInIndexer()));

        // Don't add arm, wrist, or note sensor because IntakeCycle doesn't move thems
        addRequirements(intake, indexer);
    }
}
