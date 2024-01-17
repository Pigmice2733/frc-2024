// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;
import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.DriveWithJoysticksSwerve;
import com.pigmice.frc.lib.pathfinder.Pathfinder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.ClimberConfig.ClimberState;
import frc.robot.commands.actions.intake.IntakeFromGround;
import frc.robot.commands.actions.shooter.FireIntoAmp;
import frc.robot.commands.actions.shooter.FireIntoSpeaker;
import frc.robot.commands.semi_auto.ClimbSA;
import frc.robot.commands.semi_auto.FetchRingSA;
import frc.robot.commands.semi_auto.FindRingSA;
import frc.robot.commands.semi_auto.ScoreAmpSA;
import frc.robot.commands.semi_auto.ScoreSpeakerSA;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
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
    private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);
    private final Arm arm = new Arm();
    private final Climber climberExtension = new Climber();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Indexer indexer = new Indexer();
    private final Wrist wrist = new Wrist();
    private final Vision vision = new Vision();
    private final NoteSensor noteSensor = new NoteSensor();

    private final XboxController driver;
    private final XboxController operator;
    private final Controls controls;

    private final Pathfinder pathfinder = null;
    private final SemiAutoManager semiAutoManager = new SemiAutoManager();

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);

        ControllerRumbler.setControllers(driver, operator);

        drivetrain.setDefaultCommand(new DriveWithJoysticksSwerve(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY,
                controls::getTurnSpeed,
                () -> true));

        configureButtonBindings();
        configureAutoChooser();
        configureSemiAutoManager();
    }

    public void onEnable() {
        arm.resetPID();
        climberExtension.resetPID();
        intake.resetPID();
        wrist.resetPID();
    }

    public void onDisable() {
        ControllerRumbler.stopBothControllers();
    }

    /** Initialize all semi auto tasks */
    private void configureSemiAutoManager() {
        semiAutoManager.addTasksToShuffleboard(
                new ClimbSA(drivetrain, pathfinder, arm, wrist, climberExtension, intake)
                        .withName("Climb"),
                new FetchRingSA(drivetrain, pathfinder, intake, indexer, arm, wrist, noteSensor)
                        .withName("Fetch Ring"),
                new FindRingSA(drivetrain, pathfinder, intake, indexer, arm, wrist, noteSensor, vision)
                        .withName("Find Ring"),
                new ScoreAmpSA(drivetrain, pathfinder, arm, wrist, shooter, indexer)
                        .withName("Score AMP"),
                new ScoreSpeakerSA(drivetrain, pathfinder, arm, wrist, shooter, indexer)
                        .withName("Score Speaker"));
    }

    private void configureAutoChooser() {
        autoChooser.addOption("Example",
                new InstantCommand().withName("Example Option"));

        // Default to doing nothing
        autoChooser.setDefaultOption("None", new InstantCommand());

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
        // Operator X (hold) - fire shooter into speaker
        new JoystickButton(operator, Button.kX.value)
                .whileTrue(new FireIntoSpeaker(arm, wrist, shooter, indexer));

        // Operator B (hold) - fire shooter into amp
        new JoystickButton(operator, Button.kB.value)
                .whileTrue(new FireIntoAmp(arm, wrist, shooter, indexer));

        // Operator A (hold) - intake a note from the ground
        new JoystickButton(operator, Button.kA.value)
                .whileTrue(new IntakeFromGround(intake, indexer, arm, wrist, noteSensor));

        // Operator Y (hold) - climber up on press then down on release
        new JoystickButton(operator, Button.kX.value)
                .onTrue(climberExtension.setTargetState(ClimberState.UP))
                .onFalse(climberExtension.setTargetState(ClimberState.DOWN));
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
