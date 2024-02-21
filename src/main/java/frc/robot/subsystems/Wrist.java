// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pigmice.frc.lib.pid_subsystem.PIDSubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.WristConfig;
import frc.robot.Constants.WristConfig.WristState;

public class Wrist extends PIDSubsystemBase {
    public Wrist() {
        super(new CANSparkMax(CANConfig.WRIST_ROTATION, MotorType.kBrushless), WristConfig.P, WristConfig.I,
                WristConfig.D, new Constraints(WristConfig.MAX_VELOCITY, WristConfig.MAX_ACCELERATION), false,
                WristConfig.MOTOR_POSITION_CONVERSION, 50, Constants.WRIST_TAB, false, false);

        addSoftwareStop(-180, 180);
    }

    /** Sets the height state of the climber */
    public Command setTargetState(WristState state) {
        return Commands.runOnce(() -> setTargetRotation(state.getPosition()));
    }

    /** Sets the rotation state to 'STOW' */
    public Command stow() {
        return setTargetState(WristState.STOW);
    }

    /** Sets the target rotation, then waits until it gets to that rotation */
    public Command goToState(WristState state) {
        return Commands.parallel(setTargetState(state), Commands.waitUntil(
                () -> atState(state)));
    }

    public boolean atState(WristState state) {
        return Math.abs(getCurrentRotation() - state.getPosition()) < WristConfig.POSITION_TOLERANCE;
    }

    public Command waitForState(WristState state) {
        return Commands.waitUntil(() -> atState(state));
    }
}
