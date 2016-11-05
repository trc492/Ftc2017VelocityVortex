package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcGamepad;

/**
 * Created by jzheng on 11/5/2016.
 */
@TeleOp(name="TestBallGate", group="3543Test")
public class FtcTeleOpTestBallGate extends FtcTeleOp implements FtcGamepad.ButtonHandler {
    private double gatePosition = RobotInfo.BALLGATE_CLOSE_POSITION;

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
                        gatePosition += 0.01;
                        super.robot.ballGate.setPosition(gatePosition);
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed) {
                        gatePosition -= 0.01;
                        super.robot.ballGate.setPosition(gatePosition);
                    }
                    break;
                default:
                    processed = false;
                    break;
            }
        }
        dashboard.displayPrintf(10, "gatePosition = %.2f",gatePosition);
        getOpModeTracer().traceInfo("gatePusher", "pusherPosition = %.2f",gatePosition);
        if (!processed)
            super.gamepadButtonEvent(gamepad,button,pressed);
    }
}
