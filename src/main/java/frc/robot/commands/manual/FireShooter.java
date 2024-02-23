// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class FireShooter extends SequentialCommandGroup {
        public FireShooter(Indexer indexer, Shooter shooter) {
                addCommands(
                                Commands.either(
                                                // Amp scoring (condition is true)
                                                Commands.sequence(Commands.parallel(indexer.indexBackward(),
                                                                shooter.spinFlywheelsBackward())),

                                                // Speaker scoring (condition is false)
                                                Commands.sequence(shooter.spinFlywheelsForward(),
                                                                indexer.indexBackward(),
                                                                Commands.parallel(
                                                                                Commands.sequence(
                                                                                                Commands.waitSeconds(
                                                                                                                AutoConfig.BACKUP_NOTE_TIME),
                                                                                                indexer.stopIndexer()),
                                                                                Commands.waitSeconds(
                                                                                                AutoConfig.SHOOTER_SPINUP_TIME)),
                                                                indexer.runForShooting()),

                                                // Condition
                                                () -> MoveKobraToPosition.currentKobraState == KobraState.AMP));

                addRequirements(indexer, shooter);
        }
}
