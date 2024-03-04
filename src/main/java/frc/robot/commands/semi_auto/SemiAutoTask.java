package frc.robot.commands.semi_auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.SemiAutoConfig;
import frc.robot.commands.semi_auto.subtasks.CompleteSemiAutoAction;
import frc.robot.commands.semi_auto.subtasks.PrepareSemiAutoAction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class SemiAutoTask extends SequentialCommandGroup {
    /** Creates a new ScoreAmpSA. */
    public SemiAutoTask(Drivetrain drivetrain, Arm arm, Wrist wrist, Intake intake, Indexer indexer,
            Shooter shooter,
            NoteSensor noteSensor, SemiAutoTaskType taskType) {

        String pathName = getPathName(taskType);

        PathPlannerPath trajectory = PathPlannerPath.fromPathFile(pathName);
        Translation2d endTranslation = trajectory.getPoint(trajectory.numPoints() -
                1).position;

        Command pathCommand = AutoBuilder.pathfindThenFollowPath(trajectory,
                DrivetrainConfig.PATH_CONSTRAINTS);

        addCommands(Commands.parallel(
                // Generates and follows a lineup trajectory
                Commands.sequence(Commands.runOnce(
                        () -> drivetrain.getSwerveDrive().resetOdometry(
                                trajectory
                                        .getPreviewStartingHolonomicPose())),
                        pathCommand),

                // Runs while the path is being followed
                Commands.sequence(
                        // Waits to be within preparation range
                        // TODO: maybe check the amount of time left in the path
                        // (if possible)
                        Commands.waitUntil(
                                () -> drivetrain.withinDistanceOfPoint(
                                        endTranslation,
                                        SemiAutoConfig.PREPARE_ACTION_DISTANCE)),

                        // Prepares for the action by moving the Kobra, climber, etc.
                        new PrepareSemiAutoAction(arm, wrist, intake, indexer, shooter, noteSensor, taskType))),

                // Runs the final action
                new CompleteSemiAutoAction(arm, wrist, intake, indexer, shooter, noteSensor, taskType));
    }

    public static String getPathName(SemiAutoTaskType taskType) {
        switch (taskType) {
            case INTAKE_SOURCE:
                return "saLineupSource";
            case SCORE_AMP:
                return "saLineupAmp";
            case SCORE_SPEAKER_CLOSE:
                return "saLineupSpeakerClose";
            case SCORE_SPEAKER_CENTER:
                return "saLineupSpeakerCenter";
            case SCORE_SPEAKER_FAR:
                return "saLineupSpeakerFar";
            case CLIMB:
                return "saLineupClimb";
        }

        return "";
    }

    public enum SemiAutoTaskType {
        INTAKE_SOURCE,
        SCORE_AMP,
        SCORE_SPEAKER_CLOSE,
        SCORE_SPEAKER_CENTER,
        SCORE_SPEAKER_FAR,
        CLIMB;
    }
}
