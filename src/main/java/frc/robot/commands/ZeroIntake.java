// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends Command {
  private final Intake intake;

  public ZeroIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.stopPID();
  }

  @Override
  public void execute() {
    intake.getMotor().set(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    intake.getMotor().set(0);
    intake.startPID();
    intake.setEncoderPosition(0);
    intake.resetPID();
  }

  @Override
  public boolean isFinished() {
    return intake.limitSwitchPressed();
  }
}
