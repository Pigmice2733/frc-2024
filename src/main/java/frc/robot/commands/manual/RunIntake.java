// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConfig;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;

public class RunIntake extends Command {
  private final Intake intake;
  private final NoteSensor noteSensor;
  // private final Indexer indexer;

  public RunIntake(Intake intake/* , Indexer indexer */, NoteSensor noteSensor) {
    this.intake = intake;
    this.noteSensor = noteSensor;
    // this.indexer = indexer;

    addRequirements(intake/* , indexer */);
  }

  @Override
  public void execute() {
    // indexer.outputToMotor(IndexerConfig.DEFAULT_SPEED);
    intake.outputToWheels(IntakeConfig.WHEELS_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    intake.outputToWheels(0);
    // indexer.outputToMotor(0);
  }

  @Override
  public boolean isFinished() {
    return noteSensor.noteInIntake();
    // return false;
  }
}
