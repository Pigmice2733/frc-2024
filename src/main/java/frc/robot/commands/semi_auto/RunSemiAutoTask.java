package frc.robot.commands.semi_auto;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SemiAutoConfig;
import frc.robot.commands.semi_auto.subtasks.CompleteSemiAutoAction;
import frc.robot.commands.semi_auto.subtasks.LineupSemiAuto;
import frc.robot.commands.semi_auto.subtasks.PrepareSemiAutoAction;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

public class RunSemiAutoTask extends Command {
    // TODO: Remove if not needed after initial testing
    /*
     * private final Drivetrain drivetrain;
     * private final Arm arm;
     * private final Wrist wrist;
     * private final Intake intake;
     * private final Indexer indexer;
     * private final Shooter shooter;
     * private final NoteSensor noteSensor;
     * private final SemiAutoTaskType taskType;
     */
    private final Supplier<Boolean> manualDriving;

    private final Command driveCommand;
    private final Command prepareCommand;
    private final Command actionCommand;

    public RunSemiAutoTask(Drivetrain drivetrain, Arm arm, Wrist wrist, Intake intake, Indexer indexer,
            Shooter shooter, NoteSensor noteSensor, SemiAutoTaskType taskType, Supplier<Boolean> manualDriving) {

        // TODO: remove if not needed after initial testing
        /*
         * this.drivetrain = drivetrain;
         * this.arm = arm;
         * this.wrist = wrist;
         * this.intake = intake;
         * this.indexer = indexer;
         * this.shooter = shooter;
         * this.noteSensor = noteSensor;
         * this.taskType = taskType;
         */

        this.manualDriving = manualDriving;

        String pathName = getPathName(taskType);

        PathPlannerPath trajectory = PathPlannerPath.fromPathFile(pathName);
        Translation2d endTranslation = trajectory.getPoint(trajectory.numPoints() -
                1).position;

        driveCommand = new LineupSemiAuto(drivetrain, trajectory).andThen(() -> pathFinished());

        prepareCommand =
                // Runs while the path is being followed
                Commands.sequence(
                        // Waits to be within preparation range
                        // TODO: maybe check the amount of time left in the path
                        // (if possible)
                        Commands.waitUntil(
                                () -> drivetrain.withinDistanceOfPoint(
                                        endTranslation,
                                        SemiAutoConfig.PREPARE_ACTION_DISTANCE)),

                        // Prepares for the action by moving the Kobra, climber, etc. TODO test actual
                        // command
                        // new PrepareSemiAutoAction(arm, wrist, intake, indexer, shooter, noteSensor,
                        // taskType)
                        Commands.runOnce(() -> System.out.println("Preparing command!")));

        actionCommand =
                // Runs while the path is being followed
                Commands.sequence(
                        // Waits to be within preparation range
                        // TODO: maybe check the amount of time left in the path
                        // (if possible)
                        Commands.waitUntil(
                                () -> drivetrain.withinDistanceOfPoint(
                                        endTranslation,
                                        SemiAutoConfig.FINAL_ACTION_DISTANCE)),

                        // Runs the final action TODO: test actual command
                        // new CompleteSemiAutoAction(arm, wrist, intake, indexer, shooter, noteSensor,
                        // taskType)

                        Commands.runOnce(() -> System.out.println("Completing action!")));

        // Don't add requirement to this command because commands this runs have them
    }

    @Override
    public void initialize() {
        actionCommand.schedule();
        prepareCommand.schedule();
    }

    @Override
    public void execute() {
        if (manualDriving.get()) {
            if (driveCommand.isScheduled())
                driveCommand.cancel();
        } else {
            if (!driveCommand.isScheduled()) {
                driveCommand.schedule();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(driveCommand, prepareCommand, actionCommand);
    }

    private void pathFinished() {
        System.out.println("Path finished!");
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
