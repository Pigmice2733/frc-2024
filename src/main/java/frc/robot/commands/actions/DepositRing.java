// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.AutoConfig;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class DepositRing extends SequentialCommandGroup {
    public DepositRing(Arm arm, Shooter shooter, ArmState armAngle) {
        // TODO: finalize
        addCommands(Commands.sequence(arm.setTargetState(armAngle), shooter.spinFlywheelsForward(),
                Commands.waitSeconds(AutoConfig.FLYWHEEL_SPINUP_TIME), shooter.spinFeederForward()));
        addRequirements(arm, shooter);
    }

}
