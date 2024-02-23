package frc.robot.commands.autonomous.subcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreFromStartAuto extends SequentialCommandGroup {
    public ScoreFromStartAuto(Intake intake, Indexer indexer, Arm arm, Wrist wrist, Shooter shooter) {
        // TODO
        addRequirements(intake, indexer, arm, wrist, shooter);
    }
}
