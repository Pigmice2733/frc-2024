// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class MoveKobraToPosition extends SequentialCommandGroup {
    public MoveKobraToPosition(Arm arm, Wrist wrist, Intake intake, ArmState armPosition, WristState wristPosition) {
        addCommands(intake.goToState(IntakeState.DOWN), wrist.goToState(WristState.STOW), arm.goToState(armPosition),
                wrist.goToState(wristPosition));

        addRequirements(arm, wrist, intake);
    }
}
