// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;

public class Climber extends SubsystemBase {
    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    /** Moves the chain hooks into position to raise the robot. */
    public Climber() {
        leftMotor = new CANSparkMax(CANConfig.LEFT_CLIMB, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(true);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setSmartCurrentLimit(40);

        rightMotor = new CANSparkMax(CANConfig.RIGHT_CLIMB, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setSmartCurrentLimit(40);

        ShuffleboardHelper.addOutput("Left Pos", Constants.CLIMBER_TAB, () -> getLeftPosition());
        ShuffleboardHelper.addOutput("Right Pos", Constants.CLIMBER_TAB, () -> getRightPosition());

        ShuffleboardHelper.addOutput("Left Out", Constants.CLIMBER_TAB, () -> leftMotor.get());
        ShuffleboardHelper.addOutput("Right Out", Constants.CLIMBER_TAB, () -> rightMotor.get());

    }

    /** Spins both of the climber motors */
    public void outputToMotors(double percent) {
        double rightOut = percent;
        double leftOut = percent;

        // TODO: climber software stop
        // if (applySoftwareStop) {
        // leftOut = Utils.applySoftwareStop(getLeftPosition(), leftOut,
        // ClimberConfig.downPosition, 0);
        // rightOut = Utils.applySoftwareStop(getRightPosition(), rightOut,
        // ClimberConfig.downPosition, 0);
        // }

        leftMotor.set(rightOut);
        rightMotor.set(leftOut);
    }

    /** @return the current encoder position */
    public double getLeftPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    /** @return the current encoder position */
    public double getRightPosition() {
        return rightMotor.getEncoder().getPosition();
    }
}
