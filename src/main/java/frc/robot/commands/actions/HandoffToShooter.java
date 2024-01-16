// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class HandoffToShooter extends SequentialCommandGroup {
    public HandoffToShooter(Intake intake, Shooter shooter, Indexer indexer) {
        addCommands(intake.setTargetState(IntakeState.UP),
                Commands.waitSeconds(AutoConfig.INTAKE_MOVE_TIME), intake.runWheelsBackward(),
                indexer.indexForward(), Commands.waitSeconds(AutoConfig.INTAKE_FEED_TIME),
                Commands.runOnce(() -> ControllerRumbler.rumblerOperator(RumbleType.kBothRumble, 0.25, 0.3)));
        addRequirements(intake, shooter);
    }
}
