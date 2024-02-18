// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.utils.Utils;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ClimberConfig;

public class Climber extends SubsystemBase {
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    /** Moves the chain hooks into position to raise the robot. */
    public Climber() {
        leftMotor = new CANSparkMax(CANConfig.LEFT_CLIMB, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(true);
        leftMotor.setSmartCurrentLimit(40);

        rightMotor = new CANSparkMax(CANConfig.RIGHT_CLIMB, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        leftMotor.setSmartCurrentLimit(40);
        rightMotor.follow(leftMotor, false);
    }

    /** Spins both of the climber motors */
    private void outputToMotors(double percent) {
        percent = Utils.applySoftwareStop(getPosition(), percent, ClimberConfig.downPosition, 0);
        leftMotor.set(percent);
    }

    /** Lifts the climber up */
    public Command extendClimber() {
        return Commands.runOnce(() -> outputToMotors(ClimberConfig.extensionSpeed));
    }

    /** For stowing the climber at the start of a match */
    public Command retractClimberSlow() {
        return Commands.runOnce(() -> outputToMotors(-ClimberConfig.extensionSpeed));
    }

    /** For actually climbing lifting the robot */
    public Command retractClimberFast() {
        return Commands.runOnce(() -> outputToMotors(ClimberConfig.climbingSpeed));
    }

    /** Stops the climber */
    public Command stopClimber() {
        return Commands.runOnce(() -> outputToMotors(0));
    }

    /** @return the current encoder position */
    public double getPosition() {
        return leftMotor.getEncoder().getPosition();
    }
}
