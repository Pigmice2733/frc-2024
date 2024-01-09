// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FetchRingSA extends SequentialCommandGroup {
    /** Drives to the human player then picks up a ring */
    public FetchRingSA() {
        addCommands(
        // TODO: implementation
        // Pathfind to a point right in front of the human player
        // Pickup a ring from the ground
        );

        addRequirements();
    }
}
