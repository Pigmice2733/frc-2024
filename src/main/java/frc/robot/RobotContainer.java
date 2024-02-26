// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;

import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.Constants.WristConfig.WristState;
import frc.robot.commands.ZeroIntake;
import frc.robot.commands.autonomous.RunAutoRoutine;
import frc.robot.commands.autonomous.RunAutoRoutine.AutoRoutine;
import frc.robot.commands.autonomous.subcommands.FireShooterAuto;
import frc.robot.commands.autonomous.subcommands.ScoreFromStartAuto;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.manual.ShootAmp;
import frc.robot.commands.manual.FireShooter;
import frc.robot.commands.manual.IntakeCycle;
import frc.robot.commands.manual.MoveKobraToPosition;
import frc.robot.commands.manual.RunIntake;
import frc.robot.commands.manual.MoveKobraToPosition.KobraState;
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
        new ZeroIntake(intake).schedule();
    }

    public void onDisable() {
        ControllerRumbler.stopBothControllers();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY, controls::getTurnSpeed));
    }

    private void configureAutoChooser() {
        // Default to doing nothing
        autoChooser.setDefaultOption("None", Commands.none());

        autoChooser.addOption("Two Center", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.TWO_CENTER));
        autoChooser.addOption("Two Close", new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor, AutoRoutine.TWO_CLOSE));
        autoChooser.addOption("Two Far",
                new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter, noteSensor, AutoRoutine.TWO_FAR));

        Constants.DRIVER_TAB.add("Auto Command", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(operator, Button.kX.value).whileTrue(new FireShooter(indexer, shooter))
                .onFalse(Commands.parallel(indexer.stopIndexer(), shooter.stopFlywheels()));

        /*
         * new JoystickButton(driver, Button.kY.value).whileTrue(
         * AutoBuilder.pathfindToPose(AutoConfig.Locations.AMP_LINEUP,
         * DrivetrainConfig.PATH_CONSTRAINTS));
         */

        // new JoystickButton(operator, Button.kY.value)
        // .onTrue(Commands.parallel(shooter.spinFlywheelsBackward(),
        // indexer.indexBackward()))
        // .onFalse(Commands.parallel(shooter.stopFlywheels(), indexer.stopIndexer()));

        new JoystickButton(operator, Button.kB.value).onTrue(new RunIntake(intake, indexer,
                arm, wrist, noteSensor));

        // new JoystickButton(operator, Button.kA.value).onTrue(new IntakeCycle(intake,
        // indexer,
        // arm, wrist, noteSensor));

        // new JoystickButton(operator, Button.kA.value)
        // .onTrue(Commands.runOnce(() -> indexer.indexForward().schedule()))
        // .onFalse(Commands.runOnce(() -> indexer.stopIndexer().schedule()));

        // #region DRIVER

        new JoystickButton(driver, Button.kX.value)
                .onTrue(Commands.runOnce(() -> drivetrain.getSwerveDrive()
                        .resetOdometry(new Pose2d())));

        /*
         * new JoystickButton(driver, Button.kA.value)
         * .whileTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath
         * .fromPathFile("lineupAmp").flipPath(),
         * DrivetrainConfig.PATH_CONSTRAINTS));
         */

        // #endregion

        // #region MANUAL

        // Speaker Position
        new POVButton(operator, 0) // up
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, KobraState.SPEAKER_CENTER, noteSensor, true));

        // Amp Position
        new POVButton(operator, 90) // right
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, KobraState.AMP, noteSensor, true));

        // Source Position
        new POVButton(operator, 270) // left
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, KobraState.SPEAKER_SIDE, noteSensor, true));

        // Stow Position
        new POVButton(operator, 180) // down
                .onTrue(new MoveKobraToPosition(arm, wrist, intake, KobraState.STOW, noteSensor, true));

        new JoystickButton(operator, Button.kLeftBumper.value)
                .onTrue(intake.goToState(IntakeState.STOW));

        new JoystickButton(operator,
                Button.kRightBumper.value).onTrue(intake.runWheelsBackward()).onFalse(intake.stopWheels());

        // TODO: Test after running indexer
        // new JoystickButton(operator, Button.kX.value).onTrue(new IntakeCycle(intake,
        // indexer, arm, wrist,
        // noteSensor));

        // // Fire Shooter (hold)
        // new JoystickButton(operator, Button.kX.value)
        // .onTrue(new FireShooter(indexer, shooter, noteSensor))
        // .onFalse(Commands.parallel(indexer.stopIndexer(),
        // shooter.stopFlywheels()));

        // Raise Climber (hold)
        new JoystickButton(operator, Button.kY.value)
                .onTrue(climber.extendClimber())
                .onFalse(climber.stopClimber());

        // Lower CLimber (hold)
        new JoystickButton(operator, Button.kA.value)
                .onTrue(climber.retractClimberFast())
                .onFalse(climber.stopClimber());

        // #endregion

        // #region SEMI-AUTO

        // TODO: uncomment when we are ready to test semi auto
        /*
         * // Hold to fire into speaker
         * new JoystickButton(operator, ControlBindings.SCORE_SPEAKER_BUTTON)
         * .onTrue(new ScoreSpeaker(drivetrain, pathfinder, arm, wrist, shooter,
         * indexer,
         * intake, noteSensor))
         * .onFalse(Commands.runOnce(() ->
         * CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to fire into amp
         * new JoystickButton(operator, ControlBindings.SCORE_AMP_BUTTON)
         * .onTrue(new ScoreAmp(drivetrain, pathfinder, arm, wrist, shooter,
         * indexer,
         * noteSensor, intake))
         * .onFalse(Commands.runOnce(() ->
         * CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to intake a note from the ground
         * new JoystickButton(operator, ControlBindings.INTAKE_GROUND_BUTTON)
         * .onTrue(new IntakeFromGround(intake, indexer, noteSensor))
         * .onFalse(Commands.runOnce(() ->
         * CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to intake a note from the source
         * new JoystickButton(operator, ControlBindings.INTAKE_SOURCE_BUTTON)
         * .onTrue(new IntakeFromSource(drivetrain, pathfinder, intake, arm,
         * wrist,
         * noteSensor, shooter))
         * .onFalse(Commands.runOnce(() ->
         * CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to climb
         * new JoystickButton(operator, ControlBindings.CLIMB_BUTTON)
         * .onTrue(new Climb(drivetrain, pathfinder, arm, wrist,
         * climberExtension,
         * intake))
         * .onFalse(Commands.runOnce(() ->
         * CommandScheduler.getInstance().cancel()));
         * 
         * // Hold to stow subsystems
         * new JoystickButton(operator, ControlBindings.STOW_BUTTON)
         * .onTrue(new Stow(intake, arm, wrist))
         * .onFalse(Commands.runOnce(() ->
         * CommandScheduler.getInstance().cancel()));
         */

        // #endregion
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        NamedCommands.registerCommand("prepIntake",
                Commands.sequence(
                        Commands.parallel(
                                new RunIntake(intake, indexer, arm, wrist, noteSensor)),
                        new MoveKobraToPosition(arm, wrist, intake, KobraState.SPEAKER_SIDE, noteSensor, false))); // TODO:
                                                                                                                   // both
                                                                                                                   // speaker
        // sides

        NamedCommands.registerCommand("prepScore",
                new MoveKobraToPosition(arm, wrist, intake, KobraState.SPEAKER_SIDE, noteSensor, true)); // TODO: both
                                                                                                         // speaker
                                                                                                         // sides
        NamedCommands.registerCommand("fireShooter", new FireShooter(indexer, shooter));

        return new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                noteSensor,
                AutoRoutine.TWO_CENTER);

        // TODO: Test auto chooser
        // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("autoTwoClose"));
    }

    public static enum AutoCommands {
        NONE, STRAIGHT_TEST, CURVED_TEST, PATHFINDING_TEST
    }
}
