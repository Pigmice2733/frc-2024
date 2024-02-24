// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ArmConfig.ArmState;

public class Arm extends PIDSubsystemBase {
    private final CANSparkMax rightMotor;

    /** Moves the shooter box up, down, and around the space above the robot. */
    public Arm() {
        super(new CANSparkMax(CANConfig.RIGHT_ARM, MotorType.kBrushless),
                ArmConfig.P, ArmConfig.I, ArmConfig.D, new Constraints(
                        ArmConfig.MAX_VELOCITY, ArmConfig.MAX_ACCELERATION),
                true, ArmConfig.MOTOR_POSITION_CONVERSION, 30,
                Constants.ARM_TAB, false, false);

        // Right motor
        rightMotor = new CANSparkMax(CANConfig.LEFT_ARM, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(40);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(getMotor(), true);

        addSoftwareStop(0, 150);

        // addLimitSwitch(0, DIOConfig.ARM_LIMIT_SWITCH, true,
        // LimitSwitchSide.NEGATIVE);
    }

    /** Sets the arm to the given state. */
    public Command setTargetState(ArmState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }

    /** Stows the arm. */
    public Command stow() {
        return setTargetState(ArmState.STOW);
    }

    /** Sets the arm's target state and waits until it gets there. */
    public Command goToState(ArmState state) {
        return Commands.parallel(setTargetState(state),
                Commands.waitUntil(() -> atState(state)));
    }

    public boolean atState(ArmState state) {
        return Math.abs(getCurrentRotation() - state.getPosition()) < ArmConfig.POSITION_TOLERANCE;
    }

    public Command waitForState(ArmState state) {
        return Commands.waitUntil(() -> atState(state));
    }
}
