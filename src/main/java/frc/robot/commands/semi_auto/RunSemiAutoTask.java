package frc.robot.commands.semi_auto;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SemiAutoConfig;
import frc.robot.commands.manual.FireShooter;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
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
    private final Drivetrain drivetrain;
    private final Supplier<Boolean> autoDriving;

    private final PathPlannerPath lineupPath;
    private Command driveCommand;

    private final Command prepareCommand;
    private final Command actionCommand;
    private final Command resetCommand;

    private boolean pathFinished;
    private boolean runningDriveCommand;

    public RunSemiAutoTask(Drivetrain drivetrain, Arm arm, Wrist wrist, Intake intake, Indexer indexer,
            Shooter shooter, NoteSensor noteSensor, SemiAutoTaskType taskType,
            Supplier<Boolean> autoDriving) {

        this.drivetrain = drivetrain;
        this.autoDriving = autoDriving;

        lineupPath = PathPlannerPath.fromPathFile(getPathName(taskType));
        Translation2d endTranslation = lineupPath.getPoint(lineupPath.numPoints() -
                1).position;

        driveCommand = generateDrivingCommand();

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

                        // Prepares for the action by moving the Kobra, climber, etc.
                        new PrepareSemiAutoAction(arm, wrist, intake, indexer, shooter, noteSensor,
                                taskType));

        actionCommand = Commands.sequence(Commands.waitUntil(
                () -> drivetrain.withinDistanceOfPoint(
                        endTranslation,
                        SemiAutoConfig.FINAL_ACTION_DISTANCE) || pathFinished),
                // TODO: use general command that will work for every auto task
                // (didn't work in testing for some reason)
                new FireShooter(indexer, shooter, noteSensor).withTimeout(3));

        resetCommand = new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.STOW, noteSensor, true);

        // Don't add requirement to this command because commands this runs have them
    }

    private Command generateDrivingCommand() {
        return new LineupSemiAuto(drivetrain, SemiAutoTaskType.SCORE_AMP).andThen(() -> pathFinished());
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(actionCommand, prepareCommand);
    }

    @Override
    public void execute() {
        if (!runningDriveCommand && autoDriving.get()) {
            driveCommand.schedule();
            runningDriveCommand = true;
        }

        if (!autoDriving.get() && runningDriveCommand) {
            driveCommand.cancel();
            runningDriveCommand = false;
        }

        /*
         * if (!autoDriving.get()) {
         * if (runningDriveCommand) {
         * System.out.println("canceled drive command");
         * driveCommand.cancel();
         * runningDriveCommand = false;
         * }
         * } else {
         * if (!runningDriveCommand) {
         * System.out.println("scheduled drive command");
         * driveCommand = generateDrivingCommand();
         * runningDriveCommand = true;
         * driveCommand.schedule();
         * }
         * }
         */
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(driveCommand, prepareCommand, actionCommand);
        ControllerRumbler.rumbleBoth(RumbleType.kBothRumble, 1, 0.5);

        resetCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !actionCommand.isScheduled();
        // return false;
    }

    private void pathFinished() {
        pathFinished = true;
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
