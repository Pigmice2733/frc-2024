// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    /**
     * Shoots notes out of one end into the speaker and dumps them out of the other
     * end into the amp. Intakes from the source into the speaker-shooting end.
     */
    public Shooter() {
        leftMotor = new CANSparkMax(CANConfig.TOP_SHOOTER, MotorType.kBrushed);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(false);
        leftMotor.setSmartCurrentLimit(40);

        rightMotor = new CANSparkMax(CANConfig.BOTTOM_SHOOTER, MotorType.kBrushed);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        rightMotor.setSmartCurrentLimit(40);
        rightMotor.follow(leftMotor);

        ShuffleboardHelper.addOutput("Motor Output", Constants.SHOOTER_TAB, leftMotor::get);

        ShuffleboardHelper.addInput("Set Speed", Constants.SHOOTER_TAB,
                (output) -> outputToFlywheels((double) output), 0);
    }

    /* Sets the output to the flywheels as a percent. */
    private void outputToFlywheels(double output) {
        leftMotor.set(output);
    }

    /* Spin the flywheels in the shooting direction. */
    public Command spinFlywheelsForward() {
        return Commands.runOnce(() -> outputToFlywheels(ShooterConfig.DEFAULT_SPEED));
    }

    /** Spin the flywheels in the intaking direction. */
    public Command spinFlywheelsBackward() {
        return Commands.runOnce(() -> outputToFlywheels(ShooterConfig.BACKWARD_SPEED));
    }

    /** Set the flywheels' output to 0. */
    public Command stopFlywheels() {
        return Commands.runOnce(() -> outputToFlywheels(0));
    }
}