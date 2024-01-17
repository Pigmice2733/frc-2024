// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreTrapSA extends SequentialCommandGroup {
    /** Scores in the trap */
    public ScoreTrapSA() {
        addCommands(
        // TODO: implementation (pretty sure we wanna do this while climbing, but there
        // might be a situation where that isn't true)
        );

        addRequirements();
    }
}
