package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcGamepad;
import ftclib.FtcGamepad.ButtonHandler;

/**
 * Created by jzheng on 11/5/2016.
 */
@TeleOp(name="TestButtonPusher", group="3543Test")
public class FtcTeleOpTestButtonPusher extends FtcTeleOp implements ButtonHandler {
    private double pusherPosition = 0.0;

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed) {
        boolean processed = false;
        if (gamepad == super.operatorGamepad)
        {
            processed = true;
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed) {
                        pusherPosition += 0.05;
                        super.robot.leftButtonPusher.setPosition(pusherPosition);
                        super.robot.rightButtonPusher.setPosition(pusherPosition);
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed) {
                        pusherPosition -= 0.05;
                        super.robot.leftButtonPusher.setPosition(pusherPosition);
                        super.robot.rightButtonPusher.setPosition(pusherPosition);
                    }
                    break;
                default:
                    processed = false;
                    break;
            }
        }
        dashboard.displayPrintf(1, "pusherPosition = %.2f",pusherPosition);
        getOpModeTracer().traceInfo("testPusher", "pusherPosition = %.2f",pusherPosition);
        if (!processed)
            super.gamepadButtonEvent(gamepad,button,pressed);
    }
}
