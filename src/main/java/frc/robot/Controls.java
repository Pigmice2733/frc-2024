package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControlBindings;
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

    LinearFilter driveSpeedYFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /** Returns the left joystick y-axis multiplied by the drive speed. */
    public double getDriveSpeedY() {
        double joystickY = MathUtil.applyDeadband(-driver.getLeftY(), threshold);

        joystickY = driveSpeedYFilter.calculate(joystickY); // input smoothing

        return joystickY * DrivetrainConfig.MAX_DRIVE_SPEED
                * (driver.getYButton() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    LinearFilter driveSpeedXFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /**
     * Returns the left joystick x-axis multiplied by the drive speed. When the Y
     * button is held, the result is multiplied by the slowmode multiplier before
     * returning.
     */
    public double getDriveSpeedX() {
        double joystickX = MathUtil.applyDeadband(-driver.getLeftX(), threshold);

        joystickX = driveSpeedXFilter.calculate(joystickX); // input smoothing

        return joystickX * DrivetrainConfig.MAX_DRIVE_SPEED
                * (driver.getYButton() ? DrivetrainConfig.SLOWMODE_MULTIPLIER : 1);
    }

    LinearFilter turnSpeedFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    /**
     * Returns the right joystick x-axis multiplied by the drive speed. When the Y
     * button is held, the result is multiplied by the slowmode multiplier before
     * returning.
     */
    public double getTurnSpeed() {
        double joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);

        joystickTurn = turnSpeedFilter.calculate(joystickTurn); // input smoothing

        return joystickTurn * DrivetrainConfig.MAX_TURN_SPEED;
    }

    public ControlsState getCurrentControlsState() {
        return new ControlsState();
    }

    public class ControlsState {
        public boolean scoreAmpPressed;
        public boolean scoreAmpReleased;

        public ControlsState() {
            this.scoreAmpPressed = driver.getRawButtonPressed(ControlBindings.SCORE_AMP_BUTTON);
            this.scoreAmpPressed = driver.getRawButtonReleased(ControlBindings.SCORE_AMP_BUTTON);
        }
    }
}
