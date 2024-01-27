// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;
import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.DriveWithJoysticksSwerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.auto_builder.AutoBuilder;
import frc.robot.auto_builder.AutoLayer;
import frc.robot.auto_builder.LayerBehavior.MultiOptionLayer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.vision.Vision;

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

    // TODO: uncomment once field map is added
    // private final Pathfinder pathfinder = new
    // Pathfinder(DrivetrainConfig.TRACK_WIDTH_METERS, "frc-2024");

    private final AutoBuilder autoBuilder = new AutoBuilder();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driver = new XboxController(0);
        operator = new XboxController(1);
        controls = new Controls(driver, operator);
        DriverStation.silenceJoystickConnectionWarning(true);

        ControllerRumbler.setControllers(driver, operator);

        drivetrain.setDefaultCommand(new DriveWithJoysticksSwerve(drivetrain,
                controls::getDriveSpeedX,
                controls::getDriveSpeedY,
                controls::getTurnSpeed,
                () -> true));

        configureButtonBindings();
        configureAutoBuilder();
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

    private void configureAutoBuilder() {
        AutoLayer startPosLayer = new AutoLayer("Start Position", AutoConfig.AutoBuilderOptions.StartPosition.class);

        AutoLayer autoTypeLayer = new AutoLayer("Auto Type", AutoConfig.AutoBuilderOptions.AutoType.class);

        autoBuilder.addLayer(startPosLayer, new MultiOptionLayer());
        autoBuilder.addLayer(autoTypeLayer, new MultiOptionLayer());
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
        // TODO: uncomment when pathfinder is set up
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
        // return autoChooser.getSelected().withTimeout(15);
        return autoBuilder.buildFullRoutine();
    }
}
