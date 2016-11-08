package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcGamepad;

/**
 * Created by jzheng on 11/5/2016.
 */
@TeleOp(name="TestShooter", group="3543Test")
public class FtcTeleOpTestShooter extends FtcTeleOp implements FtcGamepad.ButtonHandler {
    private double pullback = RobotInfo.SHOOTER_SIMPLE_PULLBACK_TARGET;
    private double oneShot = RobotInfo.SHOOTER_SIMPLE_ONESHOT_TARGET;
    private double openGate = RobotInfo.SHOOTER_SIPMLE_CLOSE_GATE_DELAY;
    private double closeGate = RobotInfo.SHOOTER_SIPMLE_CLOSE_GATE_DELAY;

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int button, boolean pressed) {
        boolean processed = false;
        if (gamepad == super.operatorGamepad)
        {
            if (pressed) {
                processed = true;
                switch (button) {
                    case FtcGamepad.GAMEPAD_DPAD_UP:
                        pullback += 10.0;
                        break;

                    case FtcGamepad.GAMEPAD_DPAD_DOWN:
                        pullback -= 10.0;
                        break;

                    case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                        oneShot += 10.0;
                        break;

                    case FtcGamepad.GAMEPAD_DPAD_LEFT:
                        oneShot -= 10.0;
                        break;

                    case FtcGamepad.GAMEPAD_RBUMPER:
                        openGate += 0.2;
                        closeGate += 0.2;
                        break;

                    case FtcGamepad.GAMEPAD_LBUMPER:
                        openGate -= 0.2;
                        closeGate -= 0.2;
                        break;

                    case FtcGamepad.GAMEPAD_A:
                        super.robot.shooter.setShooterParameters(pullback,oneShot,openGate,closeGate);
                        super.robot.shooter.fireOneShot();
                        break;

                    case FtcGamepad.GAMEPAD_B:
                        super.robot.shooter.setPowerManually(1.0);
                        break;

                    case FtcGamepad.GAMEPAD_X:
                        super.robot.shooter.fireContinuous(true);

                    default:
                        processed = false;
                        break;
                }
            }
            else {
                if ((button == FtcGamepad.GAMEPAD_X) || (button == FtcGamepad.GAMEPAD_A))
                {
                    super.robot.shooter.stop();
                    processed = true;
                }
            }
        }
        dashboard.displayPrintf(10, "pullback distance = %.2f, oneshot distance = %.2f, gate delay = %.2f",pullback,oneShot,openGate);
        getOpModeTracer().traceInfo("testShooter","pullback distance = %.2f, oneshot distance = %.2f, gate delay = %.2f",pullback,oneShot,openGate);
        if (!processed)
            super.gamepadButtonEvent(gamepad,button,pressed);
    }
}
