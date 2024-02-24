package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.subcommands.ScoreFromStartAuto;
import frc.robot.commands.manual.FireShooter;
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
            case TWO_CENTER:
                addCommands(
                        new ScoreFromStartAuto(intake, indexer, arm, wrist, shooter, true),
                        Commands.runOnce(
                                () -> drivetrain.getSwerveDrive().resetOdometry(
                                        PathPlannerPath.fromPathFile("autoTwoCenter")
                                                .getPreviewStartingHolonomicPose())),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("autoTwoCenter")),
                        Commands.waitSeconds(0.75), new FireShooter(indexer, shooter),
                        Commands.parallel(indexer.stopIndexer(), shooter.stopFlywheels()));
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
