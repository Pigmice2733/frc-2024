// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbSA extends SequentialCommandGroup {
    /** Drives to the chain and climbs */
    public ClimbSA() {
        addCommands(
        // TODO: implementation
        // Pathfind to the chain and stop in front of it
        // Climb
        );

        addRequirements();
    }
}
