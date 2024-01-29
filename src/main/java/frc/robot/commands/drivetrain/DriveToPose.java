// package frc.robot.commands.drivetrain;

// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.Constants.DrivetrainConfig;

// public class DriveToPose extends FollowPath {

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
