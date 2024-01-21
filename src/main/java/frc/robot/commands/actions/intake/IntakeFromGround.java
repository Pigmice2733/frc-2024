package frc.robot.commands.actions.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class IntakeFromGround extends SequentialCommandGroup {
    public IntakeFromGround(Intake intake, Indexer indexer, Arm arm, Wrist wrist, NoteSensor noteSensor) {
        addCommands(Commands.parallel( // Wait until...
                Commands.sequence(wrist.stow(), arm.stow()), // Arm and wrist are stowed
                intake.goToState(IntakeState.DOWN) // Intake is down
        ),
                intake.runWheelsForward(), indexer.indexForward(), // Start intake and indexer wheels
                noteSensor.waitForNoteInIndexer()); // Wait for a note to be detected

        andThen(intake.stopWheels(), indexer.stopIndexer(), intake.stow());

        addRequirements(intake, indexer, arm, wrist);
    }
}
