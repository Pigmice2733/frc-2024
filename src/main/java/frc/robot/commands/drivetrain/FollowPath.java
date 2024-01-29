// package frc.robot.commands.drivetrain;

// import com.pathplanner.lib.commands.FollowPathHolonomic;
// import com.pathplanner.lib.path.PathPlannerPath;

// import frc.robot.Constants.DrivetrainConfig;
// import frc.robot.subsystems.Drivetrain;

// public class FollowPath extends FollowPathHolonomic {

// /**
// * Use a SwerveController to follow a specified path.
// *
// * @param drivetrain a drivetrain subsystem
// * @param path a path to follow
// */
// public FollowPath(Drivetrain drivetrain, PathPlannerPath path) {
// super(path,
// () -> drivetrain.getSwerveDrive().getPose(),
// () -> drivetrain.getSwerveDrive().getRobotVelocity(),
// (chassisSpeeds) -> drivetrain.getSwerveDrive().drive(chassisSpeeds),
// DrivetrainConfig.PATH_CONFIG,
// () -> false);
// }

// // TODO: get events working
// /**
// * Use a SwerveController to follow a specified path.
// *
// * @param drivetrain a drivetrain subsystem
// * @param trajectory a path-following trajectory
// * @param eventMap commands to execute at certain events along the path
// * (configure events in Path Planner)
// */
// // public FollowPath(SwerveDrivetrain drivetrain, PathPlannerTrajectory
// // trajectory,
// // HashMap<String, Command> eventMap) {
// // config = drivetrain.config;
// // autoBuilder = new SwerveAutoBuilder(
// // drivetrain::getPose,
// // drivetrain::resetOdometry,
// // config.KINEMATICS,
// // config.PATH_DRIVE_PID_CONSTANTS,
// // config.PATH_TURN_PID_CONSTANTS,
// // (SwerveModuleState[] output) -> drivetrain.driveModuleStates(output),
// // eventMap,
// // false,
// // drivetrain);

// // addCommands(
// // drivetrain.resetOdometryCommand(trajectory.getInitialHolonomicPose()),
// // autoBuilder.fullAuto(trajectory));
// // addRequirements(drivetrain);
// // }

// // TODO: probably don't need this one
// /**
// * Use a SwerveController to follow a specified path.
// *
// * @param drivetrain a drivetrain subsystem
// * @param pathName the name of a premade path to follow
// * @param reversed reverse the robots direction
// */
// // public FollowPath(SwerveDrivetrain drivetrain, String pathName, boolean
// // reversed) {
// // this(
// // drivetrain,
// // PathPlanner.loadP\ath(pathName, drivetrain.config.PATH_CONSTRAINTS,
// // reversed));
// // }

// // TODO: get events working
// /**
// * Use a SwerveController to follow a specified path.
// *
// * @param drivetrain a drivetrain subsystem
// * @param pathName the name of a premade path to follow
// * @param eventMap commands to execute at certain events along the path
// * (configure events in Path Planner)
// * @param reversed reverse the robots direction
// */
// // public FollowPath(SwerveDrivetrain drivetrain, String pathName,
// // HashMap<String, Command> eventMap,
// // boolean reversed) {
// // this(
// // drivetrain,
// // PathPlanner.loadPath(pathName, drivetrain.config.PATH_CONSTRAINTS,
// reversed),
// // eventMap);

// // }
// }