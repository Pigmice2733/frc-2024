package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class RunAutoRoutine extends SequentialCommandGroup {
    public RunAutoRoutine(Drivetrain drivetrain, Intake intake, Arm arm, Wrist wrist, Indexer indexer,
            Shooter shooter, NoteSensor noteSensor, AutoRoutine autoRoutine) {

        switch (autoRoutine) {
            case ONE_CLOSE:
                addCommands(null);
        }

        addRequirements(drivetrain, intake, arm, wrist, indexer, shooter);
    }

    public enum AutoRoutine {
        NONE_WAY_CLOSE,
        NONE_WAY_FAR,
        ONE_CLOSE,
        ONE_CENTER,
        ONE_FAR,
        ONE_CLOSE_LEAVE,
        ONE_CENTER_LEAVE,
        ONE_FAR_LEAVE,
        TWO_CLOSE,
        TWO_CENTER,
        TWO_FAR,
        THREE_CENTER_TO_CLOSE,
        THREE_CENTER_TO_FAR,
        FOUR_CLOSE,
        FOUR_FAR,
        FIVE
    }
}
