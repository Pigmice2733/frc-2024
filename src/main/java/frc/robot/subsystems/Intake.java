// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.DIOConfig;
import frc.robot.Constants.IntakeConfig;
import frc.robot.Constants.IntakeConfig.IntakeState;

public class Intake extends PIDSubsystemBase {
    private final CANSparkMax wheelsMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);

    /**
     * Spins notes up from the ground in front of the robot and carries them to the
     * shooter box.
     */
    public Intake() {
        super(new CANSparkMax(CANConfig.INTAKE_PIVOT, MotorType.kBrushless), IntakeConfig.P, IntakeConfig.I,
                IntakeConfig.D, new Constraints(IntakeConfig.MAX_VELOCITY, IntakeConfig.MAX_ACCELERATION), false,
                IntakeConfig.MOTOR_POSITION_CONVERSION, 50, Constants.INTAKE_TAB, true, false);

        wheelsMotor.restoreFactoryDefaults();
        wheelsMotor.setInverted(true);
        wheelsMotor.setSmartCurrentLimit(80);

        ShuffleboardHelper.addOutput("Wheel Motor Output", Constants.INTAKE_TAB, () -> wheelsMotor.get());

        addSoftwareStop(-110, 30);

        addLimitSwitch(0, DIOConfig.INTAKE_LIMIT_SWITCH, true, LimitSwitchSide.POSITIVE);

        setMaxAllowedOutput(0.5);
    }

    /**
     * Sets the intake motor to a percent output (0.0 - 1.0) if no note is being
     * carried, and stops it otherwise.
     */
    public void outputToWheels(double percent) {
        wheelsMotor.set(percent);
    }

    /**
     * Spins intake wheels to intake notes if no note is being carried, and stops it
     * otherwise.
     */
    public Command runWheelsForward() {
        return Commands.runOnce(() -> outputToWheels(IntakeConfig.WHEELS_SPEED));
    }

    /** Spins intake wheels to eject notes. */
    public Command runWheelsBackward() {
        return Commands.runOnce(() -> outputToWheels(-IntakeConfig.WHEELS_SPEED));
    }

    /** Sets intake wheels to zero output. */
    public Command stopWheels() {
        return Commands.runOnce(() -> outputToWheels(0));
    }

    /** Sets the rotation state of the intake */
    public Command setTargetState(IntakeState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }

    /** Sets the rotation state to 'STOW' */
    public Command stow() {
        return setTargetState(IntakeState.STOW);
    }

    /** Sets the target rotation, then waits until it gets to that rotation */
    public Command goToState(IntakeState state) {
        return Commands.parallel(setTargetState(state), Commands.waitUntil(
                () -> atState(state)));
    }

    public boolean atState(IntakeState state) {
        return Math.abs(getCurrentRotation() - state.getPosition()) < IntakeConfig.POSITION_TOLERANCE;
    }

    public Command waitForState(IntakeState state) {
        return Commands.waitUntil(() -> atState(state));
    }
}