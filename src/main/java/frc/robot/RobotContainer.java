// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;
import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.DriveWithJoysticksSwerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.ArmConfig.ArmState;
import frc.robot.Constants.ClimberConfig.ClimberState;
import frc.robot.Constants.IntakeConfig.IntakeState;
import frc.robot.commands.actions.DepositRing;
import frc.robot.commands.actions.HandoffToShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClimberExtension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

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
    // private final SwerveDrivetrain drivetrain = new
    // SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);
    // private final Arm arm = new Arm();
    // private final ClimberExtension climberExtension = new ClimberExtension();
    // private final Intake intake = new Intake();
    // private final Shooter shooter = new Shooter();
    // private final Vision vision = new Vision();

    private final XboxController driver;
    private final XboxController operator;
    public final Controls controls;

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public SwerveDrive swerveDrive;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(Units.feetToMeters(14.5));
        } catch (IOException e) {
            e.printStackTrace();
        }

        driver = new XboxController(0);
        operator = new XboxController(1);

        controls = new Controls(driver, operator);
        ControllerRumbler.setControllers(driver, operator);

        // swerveDrive.driveFieldOriented(new ChassisSpeeds(1, 0, 0));

        // new DriveWithJoysticksSwerve(swerveDrive,
        // () -> 0,
        // () -> 0,
        // () -> 0,
        // () -> true).schedule();

        configureButtonBindings();
        configureAutoChooser();
    }

    private void configureAutoChooser() {
        autoChooser.addOption("Example",
                new InstantCommand().withName("Example Option"));

        // Default to doing nothing
        autoChooser.setDefaultOption("None", new InstantCommand());

        Constants.DRIVER_TAB.add("Auto Command", autoChooser);
    }

    public void onEnable() {
        // arm.resetPID();
    }

    public void onDisable() {
        ControllerRumbler.stopBothControllers();
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

        // // Operator B (hold) - Handoff to Shooter
        // new JoystickButton(operator, Button.kB.value)
        // .whileTrue(new HandoffToShooter(intake, shooter))
        // .onFalse(Commands.sequence(intake.stopWheels(), shooter.stopFeeder(),
        // intake.setTargetState(IntakeState.DOWN)));

        // // Operator X (hold) - fire shooter high
        // new JoystickButton(operator, Button.kX.value)
        // .whileTrue(new DepositRing(arm, shooter, ArmState.SPEAKER))
        // .onFalse(Commands.sequence(arm.setTargetState(ArmState.DOWN),
        // shooter.stopFlywheels(),
        // shooter.stopFeeder()));

        // // Operator B (hold) - fire shooter mid
        // new JoystickButton(operator, Button.kB.value)
        // .whileTrue(new DepositRing(arm, shooter, ArmState.AMP))
        // .onFalse(Commands.sequence(arm.setTargetState(ArmState.DOWN),
        // shooter.stopFlywheels(),
        // shooter.stopFeeder()));

        // // Operator Y (hold) - climber up on press down on release
        // new JoystickButton(operator, Button.kX.value)
        // .onTrue(climberExtension.setTargetState(ClimberState.UP))
        // .onFalse(climberExtension.setTargetState(ClimberState.DOWN));

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
