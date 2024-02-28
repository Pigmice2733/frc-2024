package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class IntakeFromGround extends SequentialCommandGroup {
    public IntakeFromGround(Intake intake, Indexer indexer, Arm arm, Wrist wrist, NoteSensor noteSensor) {
        addCommands(
                // Move intake out
                new MoveKobraToPosition(arm, wrist, intake, KobraState.STOW, noteSensor, false),

                // Start intake and indexer wheels
                Commands.parallel(intake.runWheelsForward(), indexer.indexForward()),

                // Wait for a note to be detected
                noteSensor.waitForNoteInIndexer(),
                Commands.parallel(intake.stopWheels(), indexer.stopIndexer()));

        addRequirements(intake, indexer, arm, wrist);
    }
}
