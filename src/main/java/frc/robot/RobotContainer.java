// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControlBindings;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.semi_auto.Climb;
import frc.robot.commands.semi_auto.IntakeFromSource;
import frc.robot.commands.semi_auto.FindRing;
import frc.robot.commands.semi_auto.IntakeFromGround;
import frc.robot.commands.semi_auto.ScoreAmp;
import frc.robot.commands.semi_auto.ScoreSpeaker;
import frc.robot.commands.semi_auto.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    // SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);
    // private final Arm arm = new Arm();
    // private final Climber climberExtension = new Climber();
    // private final Intake intake = new Intake();
    // private final Shooter shooter = new Shooter();
    // private final Indexer indexer = new Indexer();
    // private final Wrist wrist = new Wrist();
    // private final Vision vision = new Vision();
    // private final NoteSensor noteSensor = new NoteSensor();
    // public final SwerveDrivetrain drivetrain = new
    // SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);

    private final XboxController driver;
    private final XboxController operator;
    public final Controls controls;

    private final Pathfinder pathfinder = new Pathfinder(Constants.ROBOT_WIDTH * 100, "");
    // Pathfinder(DrivetrainConfig.TRACK_WIDTH_METERS, "frc-2024");

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);
        DriverStation.silenceJoystickConnectionWarning(true);

        ControllerRumbler.setControllers(driver, operator);

        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain, controls::getDriveSpeedX,
                controls::getDriveSpeedY, controls::getTurnSpeed));

        configureButtonBindings();
        configureAutoChooser();
    }

    public void periodic() {
        // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        // SmartDashboard.putNumber("FL Angle",
        // swerveDrive.getStates()[0].angle.getDegrees());
        // SmartDashboard.putNumber("FR Angle",
        // swerveDrive.getStates()[1].angle.getDegrees());
        // SmartDashboard.putNumber("BL Angle",
        // swerveDrive.getStates()[2].angle.getDegrees());
        // SmartDashboard.putNumber("BR Angle",
        // swerveDrive.getStates()[3].angle.getDegrees());

        // // SmartDashboard.putNumber("FL Target",
        // // swerveDrive.getModules()[0]..getDegrees());

        // SmartDashboard.putNumber("Heading Target",
        // SwerveDriveTelemetry.desiredChassisSpeeds[0]);

        // // swerveDrive.driveFieldOriented(new ChassisSpeeds(1, 0, 0));
        // // swerveDrive.drive();

        // // swerveDrive.getModules()[0].setAngle(0);
        // // swerveDrive.getModules()[1].setAngle(0);
        // // swerveDrive.getModules()[2].setAngle(0);
        // // swerveDrive.getModules()[3].setAngle(0);

        // swerveDrive.drive(new ChassisSpeeds(1, 0, 0));
    }

    public void onEnable() {
        // TODO: uncomment after drivetrain only testing
        // arm.resetPID();
        // climberExtension.resetPID();
        // intake.resetPID();
        // wrist.resetPID();
    }

    public void onDisable() {
        ControllerRumbler.stopBothControllers();
    }

    private void configureAutoChooser() {
        autoChooser.addOption("Example",
                new InstantCommand().withName("Example Option"));

        // Default to doing nothing
        autoChooser.setDefaultOption("None", new InstantCommand());

        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        autoChooser.addOption("Straight Path",
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("straightLineTest")));

        autoChooser.addOption("Curve Test",
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("curveTest")));

        autoChooser.addOption("Pathfinding Test", AutoBuilder.pathfindToPose(
                new Pose2d(1, 1, drivetrain.getSwerveDrive().getYaw()), DrivetrainConfig.PATH_CONSTRAINTS));

        Constants.DRIVER_TAB.add("Auto Command", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // TODO: uncomment once the fixed drivetrain is merged in
        /*
         * // Hold to fire into speaker
         * new JoystickButton(operator, ControlBindings.SCORE_SPEAKER_BUTTON)
         * .onTrue(new ScoreSpeaker(drivetrain, pathfinder, arm, wrist, shooter,
         * indexer,
         * intake, noteSensor))
         * .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to fire into amp
         * new JoystickButton(operator, ControlBindings.SCORE_AMP_BUTTON)
         * .onTrue(new ScoreAmp(drivetrain, pathfinder, arm, wrist, shooter, indexer,
         * noteSensor, intake))
         * .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to intake a note from the ground
         * new JoystickButton(operator, ControlBindings.INTAKE_GROUND_BUTTON)
         * .onTrue(new IntakeFromGround(intake, indexer, noteSensor))
         * .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to intake a note from the source
         * new JoystickButton(operator, ControlBindings.INTAKE_SOURCE_BUTTON)
         * .onTrue(new IntakeFromSource(drivetrain, pathfinder, intake, arm, wrist,
         * noteSensor, shooter))
         * .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to climb
         * new JoystickButton(operator, ControlBindings.CLIMB_BUTTON)
         * .onTrue(new Climb(drivetrain, pathfinder, arm, wrist, climberExtension,
         * intake))
         * .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to stow subsystems
         * new JoystickButton(operator, ControlBindings.STOW_BUTTON)
         * .onTrue(new Stow(intake, arm, wrist))
         * .onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel()));
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected().withTimeout(15);
    }
}
