// package frc.robot.commands;
// TODO: fix once the drivetrain is updated

// import java.util.List;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
// import
// com.pigmice.frc.lib.drivetrain.swerve.commands.path_following.FollowPathSwerve;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.Constants.DrivetrainConfig;

// public class DriveToPose extends FollowPathSwerve {

// public DriveToPose(SwerveDrivetrain drivetrain, Pose2d targetPose) {
// super(drivetrain, generateTrajectory(drivetrain, targetPose));
// }

// public static PathPlannerTrajectory generateTrajectory(SwerveDrivetrain
// drivetrain, Pose2d targetPose) {
// Pose2d currentPose = drivetrain.getPose();

// // Angle facing the end point when at the current point
// Rotation2d angleToEnd = Rotation2d.fromRadians(Math.atan2(targetPose.getY() -
// currentPose.getY(),
// targetPose.getX() - currentPose.getX()));

// PathPoint currentPoint = new PathPoint(currentPose.getTranslation(),
// angleToEnd, currentPose.getRotation());
// PathPoint endPoint = new PathPoint(targetPose.getTranslation(), angleToEnd,
// targetPose.getRotation());

// return
// PathPlanner.generatePath(DrivetrainConfig.SWERVE_CONFIG.PATH_CONSTRAINTS,
// List.of(currentPoint, endPoint));
// }
// }
