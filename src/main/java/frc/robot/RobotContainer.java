// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pigmice.frc.lib.controller_rumbler.ControllerRumbler;
import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;
import com.pigmice.frc.lib.drivetrain.swerve.commands.DriveWithJoysticksSwerve;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConfig;

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
    public final SwerveDrivetrain drivetrain = new SwerveDrivetrain(DrivetrainConfig.SWERVE_CONFIG);

    private final XboxController driver;
    private final XboxController operator;
    private final Controls controls;

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
    }

    private void configureAutoChooser() {
        autoChooser.addOption("Example",
                new InstantCommand().withName("Example Option"));

        // Default to doing nothing
        autoChooser.setDefaultOption("None", new InstantCommand());

        Constants.DRIVER_TAB.add("Auto Command", autoChooser);
    }

    public void onEnable() {

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
