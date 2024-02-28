package frc.robot.commands.autonomous.subcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class FireShooterAuto extends SequentialCommandGroup {
    public FireShooterAuto(Shooter shooter, Indexer indexer) {
        // TODO
        addRequirements(shooter, indexer);
    }
}
