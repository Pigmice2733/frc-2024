// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;

public class RunIntake extends SequentialCommandGroup {
  public RunIntake(Intake intake, Indexer indexer, NoteSensor noteSensor) {
    addCommands(intake.runWheelsForward(), indexer.indexForward(), noteSensor.waitForNoteInIndexer(),
        Commands.waitSeconds(AutoConfig.EXTRA_INDEX_TIME), intake.stopWheels(), indexer.stopIndexer());
    addRequirements(intake, indexer);
  }
}
