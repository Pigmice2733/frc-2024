// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class FireShooter extends SequentialCommandGroup {
  public FireShooter(Arm arm, Wrist wrist, Shooter shooter, Indexer indexer, ArmState armState, WristState wristState) {
    addCommands(shooter.spinFlywheelsForward(), // Start the flywheels
        Commands.parallel( // Wait until...
            Commands.waitSeconds(AutoConfig.SHOOTER_SPINUP_TIME), // The flywheels are at speed and...
            Commands.sequence(wrist.goToState(wristState), arm.goToState(armState)) // arm and wrist get to states
        ),
        indexer.indexForward()); // Finally, index the note to shoot

    andThen(arm.stow(), wrist.stow(),
        shooter.stopFlywheels(), indexer.stopIndexer());

    addRequirements(arm, wrist, shooter, indexer);

  }
}
