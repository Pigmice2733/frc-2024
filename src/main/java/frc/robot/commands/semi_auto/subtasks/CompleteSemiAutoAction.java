package frc.robot.commands.semi_auto.subtasks;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.manual.FireShooter;
import frc.robot.commands.manual.RunIntake;
import frc.robot.commands.semi_auto.RunSemiAutoTask.SemiAutoTaskType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class CompleteSemiAutoAction extends SequentialCommandGroup {
    public CompleteSemiAutoAction(Arm arm, Wrist wrist, Intake intake, Indexer indexer,
            Shooter shooter, NoteSensor noteSensor, SemiAutoTaskType taskType) {

        if (taskType == SemiAutoTaskType.INTAKE_SOURCE) {
            addCommands(new RunIntake(intake, indexer, arm, wrist, shooter, noteSensor));
            addRequirements();
            return;
        } else if (taskType == SemiAutoTaskType.CLIMB) {
            // TODO: automatically run a climb cycle
        } else {
            addCommands(new FireShooter(indexer, shooter, noteSensor));
            addRequirements();
        }

        addRequirements(arm, wrist, intake, indexer, shooter);
    }
}
