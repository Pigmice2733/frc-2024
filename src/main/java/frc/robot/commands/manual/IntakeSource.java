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

public class IntakeSource extends SequentialCommandGroup {
    public IntakeSource(Intake intake, Indexer indexer, Arm arm, Wrist wrist, Shooter shooter, NoteSensor noteSensor) {
        addCommands(
                // Stow kobra, keeping intake down
                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SOURCE, false),
                Commands.print("Starting wheels"),

                // Start the intake + indexer
                Commands.parallel(shooter.spinFlywheelsBackward()),

                // Wait for a note in the shooter
                noteSensor.waitForNoteInShooter(),
                noteSensor.waitForNoNoteInShooter(),
                Commands.print("Stopping wheels"),

                // Stop all the wheels
                shooter.stopFlywheels());
        addRequirements(indexer);
    }
}
