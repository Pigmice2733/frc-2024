// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConfig;
import frc.robot.Constants.IntakeConfig;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private final Intake intake;
  private final Indexer indexer;

  public RunIntake(Intake intake, Indexer indexer) {
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(intake/* , indexer */);
  }

  @Override
  public void initialize() {
    intake.outputToMotor(IntakeConfig.WHEELS_SPEED);
    indexer.outputToMotor(IndexerConfig.DEFAULT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.outputToMotor(0);
    indexer.outputToMotor(0);
  }

  @Override
  public boolean isFinished() {
    // TODO: note sensor
    return false;
  }
}
