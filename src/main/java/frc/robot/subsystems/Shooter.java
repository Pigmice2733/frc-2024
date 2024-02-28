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
    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;

    /** Controls the flywheels on the far end of the box for shooting */
    public Shooter() {
        topMotor = new CANSparkMax(CANConfig.TOP_SHOOTER, MotorType.kBrushed);
        topMotor.restoreFactoryDefaults();
        topMotor.setInverted(false);
        topMotor.setSmartCurrentLimit(100);

        bottomMotor = new CANSparkMax(CANConfig.BOTTOM_SHOOTER, MotorType.kBrushed);
        bottomMotor.restoreFactoryDefaults();
        bottomMotor.setInverted(false);
        bottomMotor.setSmartCurrentLimit(100);
        bottomMotor.follow(topMotor);

        ShuffleboardHelper.addOutput("Motor Output", Constants.SHOOTER_TAB, () -> topMotor.get());
    }

    /* Sets the output to the flywheels as a percent. */
    private void outputToFlywheels(double output) {
        topMotor.set(-output);
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