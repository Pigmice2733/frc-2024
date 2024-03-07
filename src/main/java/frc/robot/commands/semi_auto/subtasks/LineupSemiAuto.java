package frc.robot.commands.semi_auto.subtasks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.commands.semi_auto.RunSemiAutoTask;
import frc.robot.commands.semi_auto.RunSemiAutoTask.SemiAutoTaskType;
import frc.robot.subsystems.Drivetrain;

public class LineupSemiAuto extends SequentialCommandGroup {
    public LineupSemiAuto(Drivetrain drivetrain, SemiAutoTaskType taskType) {
        String pathName = RunSemiAutoTask.getPathName(taskType);

        PathPlannerPath trajectory = PathPlannerPath.fromPathFile(pathName);

        /*
         * Command pathCommand = AutoBuilder.pathfindToPose(new Pose2d(1.82, 7.47, new
         * Rotation2d(-90)),
         * DrivetrainConfig.PATH_CONSTRAINTS);
         */

        Command pathCommand = AutoBuilder.pathfindThenFollowPath(trajectory,
                DrivetrainConfig.PATH_CONSTRAINTS);

        addCommands(Commands.sequence(/*
                                       * Commands.runOnce(
                                       * () -> drivetrain.getSwerveDrive().resetOdometry(
                                       * trajectory
                                       * .getPreviewStartingHolonomicPose())),
                                       */
                pathCommand));
        addRequirements(drivetrain);
    }

    public LineupSemiAuto(Drivetrain drivetrain, PathPlannerPath trajectory) {
        Command pathCommand = AutoBuilder.pathfindThenFollowPath(trajectory,
                DrivetrainConfig.PATH_CONSTRAINTS);

        addCommands(Commands.sequence(Commands.runOnce(
                () -> drivetrain.getSwerveDrive().resetOdometry(
                        trajectory
                                .getPreviewStartingHolonomicPose())),
                pathCommand));
        addCommands();
        addRequirements(drivetrain);
    }
}
