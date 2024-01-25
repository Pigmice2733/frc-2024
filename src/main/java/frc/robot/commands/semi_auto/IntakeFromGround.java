package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;

public class IntakeFromGround extends SequentialCommandGroup {
    public IntakeFromGround(Intake intake, Indexer indexer, NoteSensor noteSensor) {
        addCommands(
                // Move intake out
                intake.goToState(IntakeState.DOWN),
                // Start intake and indexer wheels
                Commands.parallel(intake.runWheelsForward(), indexer.indexForward()),
                // Wait for a note to be detected
                noteSensor.waitForNoteInIndexer(),
                Commands.parallel(intake.stopWheels(), indexer.stopIndexer()));

        addRequirements(intake, indexer);
    }
}
