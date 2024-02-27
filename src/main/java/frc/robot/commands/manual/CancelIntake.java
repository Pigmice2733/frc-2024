// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class CancelIntake extends SequentialCommandGroup {
    public CancelIntake(Intake intake, Indexer indexer) {
        addCommands(
                intake.stopWheels(), indexer.stopIndexer());
        addRequirements(indexer);
    }
}
