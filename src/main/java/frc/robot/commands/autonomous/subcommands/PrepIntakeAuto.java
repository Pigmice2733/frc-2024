package frc.robot.commands.autonomous.subcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Wrist;

public class PrepIntakeAuto extends SequentialCommandGroup {
    public PrepIntakeAuto(Intake intake, Indexer indexer, Arm arm, Wrist wrist, NoteSensor noteSensor) {
        // TODO
        addRequirements(intake, indexer, arm, wrist);
    }
}
