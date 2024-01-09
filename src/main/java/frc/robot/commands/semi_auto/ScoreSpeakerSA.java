// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreSpeakerSA extends SequentialCommandGroup {
    /** Drives to the speaker then scores */
    public ScoreSpeakerSA() {
        addCommands(
        // TODO: implementation
        // Pathfind to a point right in front of the speaker
        // Score on the speaker
        );

        addRequirements();
    }
}
