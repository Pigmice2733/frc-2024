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
        // private final Vision vision = new Vision();

        private final XboxController driver;
        private final XboxController operator;
        public final Controls controls;

        private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();;

        /**
         * The container for the robot. Contains subsystems, OI devices, and
         * commands.
         */
        public RobotContainer() {
                drivetrain = new Drivetrain(null);

                driver = new XboxController(0);
                operator = new XboxController(1);
                controls = new Controls(driver, operator);

                DriverStation.silenceJoystickConnectionWarning(true);
                ControllerRumbler.setControllers(driver, operator);

                // Change to HIGH for debug info about swerve modules
                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;

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

                // TODO: test this and see if the drive team likes it better
                // B - press to toggle in intake cycle command
                /*
                 * new JoystickButton(operator, Button.kB.value).toggleOnTrue(
                 * new IntakeCycle(intake, indexer, arm, wrist, noteSensor));
                 */

                // POV UP - press for center speaker position
                new POVButton(operator, 0)
                                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                                KobraState.SPEAKER_CENTER,
                                                noteSensor, false));

                // POV LEFT - press for side speaker position
                new POVButton(operator, 270) // left
                                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                                KobraState.SPEAKER_SIDE,
                                                noteSensor,
                                                false));

                // POV RIGHT - press for amp position
                new POVButton(operator, 90) // right
                                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.AMP,
                                                noteSensor,
                                                false));

                // POV DOWN - press for stow position
                new POVButton(operator, 180) // down
                                .onTrue(new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.STOW,
                                                noteSensor,
                                                false));

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
                                                Commands.parallel(
                                                                new RunIntake(intake, indexer, arm, wrist, shooter,
                                                                                noteSensor)),
                                                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                                                KobraState.SPEAKER_CENTER,
                                                                noteSensor, false)));

                NamedCommands.registerCommand("prepIntakeSide",
                                Commands.sequence(
                                                Commands.parallel(
                                                                new RunIntake(intake, indexer, arm, wrist, shooter,
                                                                                noteSensor)),
                                                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter,
                                                                KobraState.SPEAKER_SIDE,
                                                                noteSensor, false)));

                NamedCommands.registerCommand("prepIntake",
                                new RunIntake(intake, indexer, arm, wrist, shooter, noteSensor));

                NamedCommands.registerCommand("prepScoreCenter",
                                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_CENTER,
                                                noteSensor,
                                                true));

                NamedCommands.registerCommand("prepScoreSide",
                                new MoveKobraToPosition(arm, wrist, intake, indexer, shooter, KobraState.SPEAKER_SIDE,
                                                noteSensor,
                                                true));

                NamedCommands.registerCommand("fireShooter", new FireShooter(indexer, shooter, noteSensor));
        }

        /** Use this to pass the autonomous command to the main {@link Robot} class. */
        public Command getAutonomousCommand() {
                /*
                 * return new RunAutoRoutine(drivetrain, intake, arm, wrist, indexer, shooter,
                 * noteSensor,
                 * AutoRoutine.TWO_CENTER);
                 */

                // TODO: Test auto chooser - might be responsible for the intake not running
                return autoChooser.getSelected();
        }

        public static enum AutoCommands {
                NONE, STRAIGHT_TEST, CURVED_TEST, PATHFINDING_TEST
        }
}
