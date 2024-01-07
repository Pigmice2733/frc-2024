// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.pigmice.frc.lib.shuffleboard_helper.ShuffleboardHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.IntakeConfig;
import frc.robot.Constants.IntakeConfig.IntakeState;

public class Intake extends PIDSubsystemBase {
    private final CANSparkMax wheelsMotor = new CANSparkMax(CANConfig.INTAKE_WHEELS, MotorType.kBrushless);

    public Intake() {
        super(new CANSparkMax(CANConfig.INTAKE_PIVOT, MotorType.kBrushless), IntakeConfig.P, IntakeConfig.I,
                IntakeConfig.D, new Constraints(IntakeConfig.MAX_VELOCITY, IntakeConfig.MAX_ACCELERATION), false,
                IntakeConfig.MOTOR_POSITION_CONVERSION, 50, Constants.INTAKE_TAB, true);

        wheelsMotor.restoreFactoryDefaults();
        wheelsMotor.setInverted(false);

        ShuffleboardHelper.addOutput("Motor Output", Constants.INTAKE_TAB, () -> wheelsMotor.get());
    }

    /** Sets the intake motor to a percent output (0.0 - 1.0) */
    public void outputToMotor(double percent) {
        wheelsMotor.set(percent);
    }

    /** Spins intake wheels to intake balls. */
    public Command runWheelsForward() {
        return Commands.runOnce(() -> outputToMotor(IntakeConfig.WHEELS_SPEED));
    }

    /** Spins intake wheels to eject balls. */
    public Command runWheelsBackward() {
        return Commands.runOnce(() -> outputToMotor(-IntakeConfig.WHEELS_SPEED));
    }

    /** Sets intake wheels to zero output. */
    public Command stopWheels() {
        return Commands.runOnce(() -> outputToMotor(0));
    }

    /** Sets the rotation state of the intake */
    public Command setTargetState(IntakeState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }
}