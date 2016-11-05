package team3543;

import ftclib.FtcGamepad;

/**
 * Created by jzheng on 11/5/2016.
 */

public class FtcTeleOpTestShooter extends FtcTeleOp implements FtcGamepad.ButtonHandler {
    private double highThreshold = RobotInfo.SHOOTER_SPEED_HIGH_THRESHOLD;
    private double lowThreshold = RobotInfo.SHOOTER_SPEED_LOW_THRESHOLD;
    private double pullBackTarget = RobotInfo.SHOOTER_PULLBACK_TARGET;

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed) {
        boolean processed = false;
        if ((gamepad == super.operatorGamepad) && pressed)
        {
            if (pressed) {
                processed = true;
                switch (button) {
                    case FtcGamepad.GAMEPAD_DPAD_UP:
                        highThreshold += 1.0;
                        break;

                    case FtcGamepad.GAMEPAD_DPAD_DOWN:
                        highThreshold -= 1.0;
                        break;

                    case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                        lowThreshold += 1.0;
                        break;

                    case FtcGamepad.GAMEPAD_DPAD_LEFT:
                        lowThreshold -= 1.0;
                        break;

                    case FtcGamepad.GAMEPAD_X:
                        pullBackTarget += 1.0;
                        break;

                    case FtcGamepad.GAMEPAD_Y:
                        pullBackTarget -= 1.0;
                        break;

                    case FtcGamepad.GAMEPAD_A:
                        super.robot.shooter.setShooterParameter(lowThreshold, highThreshold, pullBackTarget);
                        super.robot.shooter.fireOneShot();
                        break;

                    case FtcGamepad.GAMEPAD_B:
                        super.robot.shooter.setPowerManually(1.0);
                        break;
                    default:
                        processed = false;
                        break;
                }
            }
            else {
                if (button == FtcGamepad.GAMEPAD_B)
                {
                    super.robot.shooter.setPowerManually(0.0);
                    processed = true;
                }
            }
        }
        dashboard.displayPrintf(1, "hThreshold = %.2f, lThreshold = %.2f, target = %.2f",highThreshold,lowThreshold,pullBackTarget);
        getOpModeTracer().traceInfo("testShooter","highThreshold = %.2f, lowThreshold = %.2f, target = %.2f",highThreshold,lowThreshold,pullBackTarget);
        if (!processed)
            super.gamepadButtonEvent(gamepad,button,pressed);
    }
}
