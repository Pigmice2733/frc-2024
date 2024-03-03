package frc.robot.commands.semi_auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreAmpSA extends SequentialCommandGroup {
    /** Creates a new ScoreAmpSA. */
    public ScoreAmpSA(Drivetrain drivetrain, Arm arm, Wrist wrist, Indexer indexer, Shooter shooter,
            NoteSensor noteSensor) {

        addCommands(
                Commands.parallel(
                // Lineup with the amp and...

                // Wait to be within range, then raise the arm

                )

        // Fire the shooter

        );

        addRequirements(drivetrain, arm, wrist, shooter);
    }
}
