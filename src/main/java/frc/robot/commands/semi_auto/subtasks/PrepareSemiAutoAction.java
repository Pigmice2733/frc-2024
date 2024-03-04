package frc.robot.commands.semi_auto.subtasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.commands.semi_auto.SemiAutoTask.SemiAutoTaskType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class PrepareSemiAutoAction extends SequentialCommandGroup {
    public PrepareSemiAutoAction(Arm arm, Wrist wrist, Intake intake, Indexer indexer,
            Shooter shooter, NoteSensor noteSensor, SemiAutoTaskType taskType) {
        switch (taskType) {
            case INTAKE_SOURCE:
                addCommands(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.STOW, noteSensor,
                        false));
                break;
            case SCORE_AMP:
                addCommands(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.AMP, noteSensor,
                        true));
                break;
            case SCORE_SPEAKER_CLOSE:
                addCommands(
                        new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_SIDE,
                                noteSensor, true));
                break;
            case SCORE_SPEAKER_CENTER:
                addCommands(
                        new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_CENTER,
                                noteSensor, true));
                break;
            case SCORE_SPEAKER_FAR:
                addCommands(
                        new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_SIDE,
                                noteSensor, true));
                break;
            case CLIMB:
                // TODO: raise the climber, but only when it wont collide with the stage
                break;
        }
        addRequirements(arm, wrist, intake, indexer, shooter);
    }
}
