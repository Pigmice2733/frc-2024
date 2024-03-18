// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;

import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.commands.autonomous.RunAutoRoutine;
import frc.robot.commands.autonomous.RunAutoRoutine.AutoRoutine;
import frc.robot.commands.autonomous.subcommands.ScoreFromStartAuto;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.manual.CancelIntake;
import frc.robot.commands.manual.FireShooter;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.RunClimber;
import frc.robot.commands.manual.RunIntake;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
import frc.robot.commands.semi_auto.RunSemiAutoTask;
import frc.robot.commands.semi_auto.RunSemiAutoTask.SemiAutoTaskType;
import frc.robot.commands.semi_auto.subtasks.LineupSemiAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.vision.Vision;

import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

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
    private final Drivetrain drivetrain;
    private final Arm arm = new Arm();
    private final Climber climber = new Climber();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Indexer indexer = new Indexer();
    private final Wrist wrist = new Wrist();
    private final NoteSensor noteSensor = new NoteSensor();
    private final Vision vision = new Vision();

    private final XboxController driver;
    private final XboxController operator;
    public final Controls controls;

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        drivetrain = new Drivetrain(vision);

        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);

        DriverStation.silenceJoystickConnectionWarning(true);
        ControllerRumbler.setControllers(driver, operator);

        // Change to HIGH for debug info about swerve modules
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        configureDefaultCommands();
        configureButtonBindings();

        configureNamedCommands();
        configureAutoChooser();
    }

    public void teleopPeriodic() {
    }

    public void onEnable() {
        arm.resetPID();
        intake.resetPID();
        wrist.resetPID();
    }

    public void teleopEnable() {
        // new ZeroIntake(intake).schedule();
    }

    public void onDisable() {
        ControllerRumbler.stopBothControllers();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY, controls::getTurnSpeed));

        climber.setDefaultCommand(new RunClimber(climber, controls::getClimberSpeed));
    }

    private void configureAutoChooser() {
        // Default to doing nothing
        autoChooser.setDefaultOption("None", Commands.none());

        autoChooser.addOption("Just Shoot Center",
                new ScoreFromStartAuto(intake, indexer, arm, wrist, shooter, true,
                        KobraState.SPEAKER_CENTER, noteSensor));

        autoChooser.addOption("Just Shoot Side",
                new ScoreFromStartAuto(intake, indexer, arm, wrist, shooter, true,
                        KobraState.SPEAKER_SIDE, noteSensor));

        autoChooser.addOption("One Close", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.ONE_CLOSE));

        autoChooser.addOption("One Center", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.ONE_CENTER));

        autoChooser.addOption("One Far", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.ONE_FAR));

        autoChooser.addOption("Two Center", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.TWO_CENTER));

        autoChooser.addOption("Two Close", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.TWO_CLOSE));

        autoChooser.addOption("Two Far", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.TWO_FAR));

        autoChooser.addOption("Three Center to Close",
                new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                        noteSensor, AutoRoutine.THREE_CENTER_TO_CLOSE));
        autoChooser.addOption("Three Center to Far",
                new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                        noteSensor, AutoRoutine.THREE_CENTER_TO_FAR));

        Constants.DRIVER_TAB.add("Auto Command", autoChooser).withPosition(0, 0);
    }

    /** Use this method to define your button->command mappings. */
    private void configureButtonBindings() {
        /*
         * DRIVER CONTROLS
         */

        // X - press to reset odometry
        new JoystickButton(driver, Button.kX.value)
                .onTrue(Commands.runOnce(() -> drivetrain.getSwerveDrive()
                        .resetOdometry(new Pose2d())));

        // POV UP - lineup speaker center
        new POVButton(driver, 0) // up
                .whileTrue(new LineupSemiAuto(drivetrain, SemiAutoTaskType.SCORE_SPEAKER_CENTER));

        // TODO: lineup speaker left/right based on alliance color from driverstation

        /**
         * Auto lineup command (does not automatically score)
         */

        // Y - lineup speaker center
        new JoystickButton(driver, Button.kY.value)
                .whileTrue(new LineupSemiAuto(drivetrain, SemiAutoTaskType.SCORE_SPEAKER_CENTER));

        // X - lineup speaker side
        new JoystickButton(driver, Button.kX.value)
                .whileTrue(new LineupSemiAuto(drivetrain, SemiAutoTaskType.SCORE_SPEAKER_FAR));

        // B - lineup amp
        new JoystickButton(driver, Button.kB.value)
                .whileTrue(new LineupSemiAuto(drivetrain, SemiAutoTaskType.SCORE_AMP));

        // A - lineup climb
        new JoystickButton(driver, Button.kA.value)
                .whileTrue(new LineupSemiAuto(drivetrain, SemiAutoTaskType.CLIMB));

        /**
         * Semi Auto Commands
         */

        // POV UP - score speaker center semi auto
        new POVButton(driver, 0) // up
                .onTrue(new RunSemiAutoTask(drivetrain, arm, wrist, intake, indexer, shooter,
                        noteSensor,
                        SemiAutoTaskType.SCORE_SPEAKER_CENTER, () -> driver.getLeftBumper(),
                        () -> driver.getXButton()));

        // POV LEFT - score speaker side semi auto
        new POVButton(driver, 270) // left
                .onTrue(new RunSemiAutoTask(drivetrain, arm, wrist, intake, indexer, shooter,
                        noteSensor,
                        SemiAutoTaskType.SCORE_SPEAKER_FAR, () -> driver.getAButton(),
                        () -> driver.getXButton()));

        // POV RIGHT - score amp semi auto
        new POVButton(driver, 90) // right
                .onTrue(new RunSemiAutoTask(drivetrain, arm, wrist, intake, indexer, shooter,
                        noteSensor,
                        SemiAutoTaskType.SCORE_AMP, () -> driver.getAButton(),
                        () -> driver.getXButton()));

        // POV DOWN - intake from source semi auto
        new POVButton(driver, 180) // down
                .onTrue(new RunSemiAutoTask(drivetrain, arm, wrist, intake, indexer, shooter,
                        noteSensor,
                        SemiAutoTaskType.CLIMB, () -> driver.getAButton(),
                        () -> driver.getXButton()));

        /*
         * OPERATOR CONTROLS
         */

        // RIGHT BUMPER - hold to fire the shooter (amp or speaker)
        new JoystickButton(operator, Button.kRightBumper.value)
                .whileTrue(new FireShooter(indexer, shooter, noteSensor))
                .onFalse(Commands.parallel(indexer.stopIndexer(), shooter.stopFlywheels()));

        // B - press to toggle the intake
        new JoystickButton(operator, Button.kB.value).onTrue(
                new RunIntake(intake, indexer, arm, wrist, shooter, noteSensor));

        // X - cancel the intake cycle
        new JoystickButton(operator, Button.kX.value).onTrue(new CancelIntake(intake, indexer));

        // Y - stow the intake
        new JoystickButton(operator, Button.kY.value).onTrue(intake.setTargetState(IntakeState.STOW));

        // POV UP - press for center speaker position
        new POVButton(operator, 0) // up
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                        KobraState.SPEAKER_CENTER,
                        false));

        // POV LEFT - press for side speaker position
        new POVButton(operator, 270) // left
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                        KobraState.SPEAKER_SIDE,

                        false));

        // POV RIGHT - press for amp position
        new POVButton(operator, 90) // right
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.AMP,

                        false));

        // POV DOWN - press for stow position
        new POVButton(operator, 180) // down
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.STOW,

                        true));

        // A - hold to run the intake and indexer backward
        new JoystickButton(operator, Button.kA.value)
                .onTrue(intake.runWheelsBackward())
                .onFalse(intake.stopWheels());
    }

    /** Configures the named commands used in path planner */
    private void configureNamedCommands() {
        // TODO: both speaker sides
        NamedCommands.registerCommand("prepIntakeCenter",
                Commands.sequence(
                        Commands.print("PREP INTAKE CENTER"),
                        Commands.parallel(
                                new RunIntake(intake, indexer, arm, wrist, shooter,
                                        noteSensor))
                                .withTimeout(2),
                        new CancelIntake(intake, indexer), // TODO: should not be needed
                        new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                KobraState.SPEAKER_CENTER,
                                false)));

        NamedCommands.registerCommand("prepIntakeSide",
                Commands.sequence(
                        Commands.print("PREP INTAKE SIDE"),
                        Commands.parallel(
                                new RunIntake(intake, indexer, arm, wrist, shooter,
                                        noteSensor))
                                .withTimeout(2),
                        new CancelIntake(intake, indexer), // TODO: should not be needed
                        new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                KobraState.SPEAKER_SIDE,
                                false)));

        NamedCommands.registerCommand("prepIntake",
                new RunIntake(intake, indexer, arm, wrist, shooter, noteSensor));

        NamedCommands.registerCommand("prepScoreCenter",
                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_CENTER,
                        true));

        NamedCommands.registerCommand("prepScoreSide",
                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_SIDE,
                        true));

        // TODO: diff for side
        NamedCommands.registerCommand("fireShooter",
                Commands.sequence(Commands.print("FIRE SHOOTER"),
                        new ScoreFromStartAuto(intake, indexer, arm, wrist, shooter,
                                true, KobraState.SPEAKER_CENTER, noteSensor)));
    }

    /** Use this to pass the autonomous command to the main {@link Robot} class. */
    public Command getAutonomousCommand() {
        /*
         * return new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
         * noteSensor,
         * AutoRoutine.TWO_CENTER);
         */

        // TODO: Test auto chooser - might be responsible for the intake not running

        /*
         * return Commands.sequence(Commands.runOnce(
         * () -> drivetrain.getSwerveDrive().resetOdometry(
         * PathPlannerPath.fromPathFile("autoTwoClose")
         * .getPreviewStartingHolonomicPose())),
         * AutoBuilder.followPath(PathPlannerPath.fromPathFile("autoTwoClose")));
         */
        return autoChooser.getSelected();
    }
}
