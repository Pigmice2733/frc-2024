// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends ParallelCommandGroup {
    public ShootAmp(Indexer indexer, Shooter shooter) {
        addCommands(indexer.indexBackward(), shooter.spinFlywheelsBackward());

        addRequirements(indexer, shooter);
    }
}
