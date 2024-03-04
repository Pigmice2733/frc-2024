package frc.robot.commands.semi_auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.SemiAutoConfig;
import frc.robot.commands.manual.FireShooter;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class ScoreAmpSA extends SequentialCommandGroup {
    /** Creates a new ScoreAmpSA. */
    public ScoreAmpSA(Drivetrain drivetrain, Arm arm, Wrist wrist, Intake intake, Indexer indexer, Shooter shooter,
            NoteSensor noteSensor) {

        PathPlannerPath trajectory = PathPlannerPath.fromPathFile("lineupAmp");
        Translation2d endTranslation = trajectory.getPoint(trajectory.numPoints() - 1).position;

        Command pathCommand = AutoBuilder.pathfindThenFollowPath(trajectory, DrivetrainConfig.PATH_CONSTRAINTS);

        addCommands(
                Commands.parallel(
                        // Lineup with the amp
                        Commands.sequence(Commands.runOnce(
                                () -> drivetrain.getSwerveDrive().resetOdometry(
                                        trajectory
                                                .getPreviewStartingHolonomicPose())),
                                pathCommand),

                        // Runs while the path is being followed
                        Commands.sequence(
                                // Wait to be within range
                                // TODO: maybe check the amount of time left on the path (if possible)
                                Commands.waitUntil(() -> drivetrain.withinDistanceOfPoint(
                                        endTranslation, SemiAutoConfig.AMP_RAISE_ARM_DISTANCE)),

                                // Raise the arm
                                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                        KobraState.AMP, noteSensor, true))),

                // Fire into the amp
                new FireShooter(indexer, shooter, noteSensor));

        addRequirements(drivetrain, arm, wrist, shooter);
    }
}
