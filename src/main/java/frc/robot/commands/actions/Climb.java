package frc.robot.commands.actions;

import com.pigmice.frc.lib.drivetrain.swerve.SwerveDrivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConfig;
import frc.robot.Constants.ClimberConfig.ClimberState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class Climb extends SequentialCommandGroup {
    /** Climbs, assuming the drivetrain is positioned in front of the chain */
    public Climb(SwerveDrivetrain drivetrain, Arm arm, Wrist wrist, Climber climber, Intake intake) {
        addCommands(
                Commands.parallel( // Wait until...
                        Commands.sequence(wrist.stow(), arm.stow()), // Arm and wrist are stowed
                        intake.stow() // Intake is down
                ),
                climber.goToState(ClimberState.UP), // Wait for climber to raise
                Commands.runOnce( // Drive forward
                        () -> drivetrain.driveChassisSpeeds(new ChassisSpeeds(0, AutoConfig.CLIMB_DRIVE_SPEED, 0))),

                Commands.waitSeconds(AutoConfig.CLIMB_DRIVE_TIME),

                Commands.runOnce( // Stop driving
                        () -> drivetrain.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0))),
                climber.goToState(ClimberState.DOWN) // Wait for climber to lower
        );

        addRequirements(drivetrain, arm, wrist, climber, intake);

        andThen(climber.setTargetState(ClimberState.DOWN));
    }
}
