package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ClimberConfig;
import frc.robot.Constants.DrivetrainConfig;

public class Controls {
    XboxController driver;
    XboxController operator;

    // If a value from a joystick is less than this, it will return 0.
    private double threshold = Constants.AXIS_THRESHOLD;

    public Controls(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    /**
     * @return the left joystick y-axis multiplied by the drive speed. When the Y
     *         button is held, the result is multiplied by the slowmode multiplier
     *         before
     *         returning.
     */
    public double getDriveSpeedY() {
        double joystickY = MathUtil.applyDeadband(-driver.getLeftY(), threshold);

        return joystickY * DrivetrainConfig.MAX_DRIVE_SPEED
                * (driver.getRightBumper() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    /**
     * @return the left joystick x-axis multiplied by the drive speed. When the Y
     *         button is held, the result is multiplied by the slowmode multiplier
     *         before
     *         returning.
     */
    public double getDriveSpeedX() {
        double joystickX = MathUtil.applyDeadband(-driver.getLeftX(), threshold);

        return joystickX * DrivetrainConfig.MAX_DRIVE_SPEED
                * (driver.getRightBumper() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    /**
     * @return the right joystick x-axis multiplied by the drive speed. When the Y
     *         button is held, the result is multiplied by the slowmode multiplier
     *         before
     *         returning.
     */
    public double getTurnSpeed() {
        double joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);

        return -joystickTurn * DrivetrainConfig.MAX_TURN_SPEED
                * (driver.getRightBumper() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    /** @return the climber speed calculated from the triggers */
    public double getClimberSpeed() {
        double leftTrigger = MathUtil.applyDeadband(operator.getLeftTriggerAxis(), threshold);
        double rightTrigger = MathUtil.applyDeadband(operator.getRightTriggerAxis(), threshold);

        return (leftTrigger - rightTrigger) * ClimberConfig.climbingSpeed;
    }
}
