// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FindRingSA extends SequentialCommandGroup {
    /** Searches for a ring on the floor and picks it up */
    public FindRingSA() {
        addCommands(
        // TODO: implementation
        // Search for a ring (maybe pathfind to a central location)
        // Once a ring is found, drive in front of it
        // Intake from ground
        );

        addRequirements();
    }
}
