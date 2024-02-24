// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConfig;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class MoveKobraToPosition extends SequentialCommandGroup {
    public MoveKobraToPosition(Arm arm, Wrist wrist, Intake intake, KobraState state) {
        addCommands(Commands.runOnce(() -> {
            currentKobraState = state;
        }));

        // Lower the intake
        addCommands(intake.goToState(IntakeState.DOWN));

        // If the wrist can go out of frame, move the arm to the wrist rotation position
        addCommands(Commands.either(
                arm.goToState(ArmState.WRIST_ROTATION),
                Commands.none(),
                () -> arm.getCurrentRotation() > (ArmState.WRIST_ROTATION.getPosition()
                        + ArmConfig.POSITION_TOLERANCE)));

        // Stow the wrist
        addCommands(wrist.goToState(WristState.STOW));

        // Move the the ending configuration
        switch (state) {
            case STOW:
                addCommands(arm.goToState(ArmState.STOW)); // Stow the arm (wrist already stowed)
                break;
            case AMP:
                addCommands(
                        Commands.parallel(arm.goToState(ArmState.AMP), // Arm to wrist rotation pos then...
                                wrist.goToState(WristState.AMP))); // Wrist to amp pos then...
                // arm.goToState(ArmState.AMP)); // Arm to amp pos
                break;
            case SOURCE:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.SOURCE), // Arm to source position and...
                        wrist.goToState(WristState.SOURCE))); // Wrist to source position
                break;
            case SPEAKER:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.SPEAKER), // Arm to speaker position and...
                        wrist.goToState(WristState.SPEAKER))); // Wrist to speaker position
                break;
            case TRAP:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.TRAP), // Arm to trap position and...
                        wrist.goToState(WristState.TRAP))); // Wrist to trap position
                break;
        }

        addRequirements(arm, wrist, intake);
    }

    public static KobraState currentKobraState;

    public enum KobraState {
        STOW,
        AMP,
        SPEAKER,
        SOURCE,
        TRAP
    }
}
