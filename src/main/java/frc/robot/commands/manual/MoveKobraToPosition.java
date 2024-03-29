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
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class MoveKobraToPosition extends SequentialCommandGroup {
    public MoveKobraToPosition(Arm arm, Wrist wrist, Intake intake, Indexer indexer, Shooter shooter, KobraState state,
            boolean stowIntake) {
        addCommands(intake.stopWheels(), indexer.stopIndexer(), shooter.stopFlywheels());
        /*
         * addCommands(new InstantCommand(() -> {
         * if (state != KobraState.STOW && !noteSensor.noteInIndexer()) {
         * this.cancel();
         * }
         * }));
         */

        addCommands(Commands.runOnce(() -> {
            currentKobraState = state;
        }));

        // Lower the intake
        addCommands(intake.goToState(IntakeState.DOWN));

        // If the wrist can go out of frame, move the arm to the wrist rotation position
        addCommands(Commands.either(
                Commands.parallel(arm.setTargetState(ArmState.WRIST_ROTATION), Commands.waitSeconds(0.05)),
                Commands.none(),
                () -> arm.getCurrentRotation() > (ArmState.WRIST_ROTATION.getPosition()
                        + ArmConfig.POSITION_TOLERANCE)));

        // Stow the wrist
        // addCommands(wrist.goToState(WristState.STOW));

        // Move the the ending configuration
        switch (state) {
            case STOW:
                addCommands(Commands.parallel(wrist.setTargetState(WristState.STOW),
                        Commands.sequence(Commands.waitSeconds(0.2),
                                arm.goToState(ArmState.STOW)))); // Stow
                // the
                // arm
                // (wrist
                // already stowed)
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
            case SPEAKER_CENTER:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.SPEAKER_CENTER), // Arm to speaker position and...
                        wrist.goToState(WristState.SPEAKER_CENTER))); // Wrist to speaker position
                break;
            case SPEAKER_SIDE:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.SPEAKER_SIDE), // Arm to speaker position and...
                        wrist.goToState(WristState.SPEAKER_SIDE))); // Wrist to speaker position
                break;
            case TRAP:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.TRAP), // Arm to trap position and...
                        wrist.goToState(WristState.TRAP))); // Wrist to trap position
                break;
            case GRAB_FROM_CHASSIS:
                addCommands(Commands.parallel( // At the same time:
                        arm.goToState(ArmState.GRAB_FROM_CHASSIS), // Arm to trap position and...
                        wrist.goToState(WristState.GRAB_FROM_CHASSIS))); // Wrist to trap position
                break;
        }

        if (stowIntake) {
            addCommands(intake.goToState(IntakeState.STOW));
        }

        addRequirements(arm, wrist, intake);
    }

    public static KobraState currentKobraState;

    public enum KobraState {
        STOW,
        AMP,
        SPEAKER_CENTER,
        SPEAKER_SIDE,
        SOURCE,
        TRAP,
        GRAB_FROM_CHASSIS
    }
}
