/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team3543;

import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class AutoBeacon implements TrcRobot.AutoStrategy
{
    private enum State
    {
        SHOOT_PARTICLES,
        GOTO_NEAR_BEACON,
        ALIGN_WALL,
        GOTO_WALL,
        FIND_LINE,
        PUSH_BUTTON,
        RETRACT,
        DONE
    }   //enum State

    private static final String moduleName = "AutoBeaconOld";

    private HalDashboard dashboard;
    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private FtcAuto.StartPosition startPos;
    private double delay;
    private int shootParticles;
    private FtcAuto.BeaconButtons beaconButtons;
    private FtcAuto.BeaconOption option;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;
    private boolean particleLoaded = true;

    public AutoBeacon(
            Robot robot,
            FtcAuto.Alliance alliance,
            FtcAuto.StartPosition startPos,
            double delay,
            int shootParticles,
            FtcAuto.BeaconButtons beaconButtons,
            FtcAuto.BeaconOption option)
    {
        this.robot = robot;
        this.dashboard = robot.dashboard;
        this.alliance = alliance;
        this.startPos = startPos;
        this.delay = delay;
        this.shootParticles = shootParticles;
        this.beaconButtons = beaconButtons;
        this.option = option;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine(moduleName);
        sm.start(State.SHOOT_PARTICLES);
    }

    @Override
    public void autoPeriodic(double elapsedTime)
    {
        //
        // Do trace logging and debug tracing.
        //
        if (robot.pidDrive.isEnabled())
        {
            robot.encoderYPidCtrl.printPidInfo(tracer);
            robot.gyroPidCtrl.printPidInfo(tracer);
            /*
            tracer.traceInfo(moduleName, "[%.3f] LineDetect: value=%d",
                             elapsedTime,
                             (Integer)robot.lineDetectionSensor.getRawData(0, TrcAnalogInput.DataType.INPUT_DATA).value);
                             */
        }

        dashboard.displayPrintf(1, moduleName + ": %s,%s,delay=%.0f,shootParticles=%d,beaconButtons=%s,option=%s",
                alliance.toString(), startPos.toString(), delay,
                shootParticles, beaconButtons.toString(), option.toString());
        dashboard.displayPrintf(2, "RGBAH: [%d,%d,%d,%d,%x]",
                robot.beaconColorSensor.red(),
                robot.beaconColorSensor.green(),
                robot.beaconColorSensor.blue(),
                robot.beaconColorSensor.alpha(),
                robot.beaconColorSensor.argb());

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            tracer.traceInfo(moduleName, "State: %s [%.3f]", state.toString(), elapsedTime);
            dashboard.displayPrintf(7, "State: %s [%.3f]", state.toString(), elapsedTime);

            switch (state)
            {
                case SHOOT_PARTICLES:
                    //
                    // Fire a particle if any.
                    //
                    if (shootParticles > 0)
                    {
                        if (particleLoaded)
                        {
                            robot.shooter.fireOneShot(event);
                            particleLoaded = false;
                        }
                        else
                        {
                            robot.shooter.loadAndFireOneShot(event);
                        }
                        shootParticles--;
                        sm.addEvent(event);
                        sm.waitForEvents(State.SHOOT_PARTICLES);
                    }
                    //
                    // Do delay if any.
                    //
                    else if (delay > 0.0 && delay - elapsedTime > 0)
                    {
                        timer.set(delay - elapsedTime, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.GOTO_NEAR_BEACON);
                    }
                    else
                    {
                        sm.setState(State.GOTO_NEAR_BEACON);
                    }
                    break;

                case GOTO_NEAR_BEACON:
                    robot.pidDrive.setTarget(0.0, 60.0, -45.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.ALIGN_WALL);
                    break;

                case ALIGN_WALL:
                    robot.pidDrive.setTarget(0.0, 0.0, 0.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    robot.pidDrive.setTarget(-15.0, 0.0, 0.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.FIND_LINE);
                    break;

                case FIND_LINE:
                    robot.lineTrigger.setEnabled(true);
                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.pidDrive.setTarget(0.0, -12.0, 0.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.PUSH_BUTTON);
                    break;

                case PUSH_BUTTON:
                    robot.lineTrigger.setEnabled(false);
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    int redValue = robot.beaconColorSensor.red();
                    int greenValue = robot.beaconColorSensor.green();
                    int blueValue = robot.beaconColorSensor.blue();
                    boolean isRed = redValue > blueValue && redValue > greenValue;
                    boolean isBlue = blueValue > redValue && blueValue > greenValue;
                    tracer.traceInfo(
                            moduleName, "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, isRed? "true": "false", isBlue? "true": "false");
                    //
                    // Determine which button to push and do it.
                    //
                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isRed ||
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isBlue)
                    {
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        leftPusherExtended = true;
                    }
                    else if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isBlue ||
                             alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isRed)
                    {
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        rightPusherExtended = true;
                    }
                    //
                    // It takes sometime for the button pusher to extend, set a timer to wait for it.
                    //
                    if (leftPusherExtended || rightPusherExtended)
                    {
                        timer.set(3.0, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.RETRACT);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case RETRACT:
                    //
                    // Release the button pusher and retract the hanging hook.
                    //
                    if (leftPusherExtended)
                    {
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                        leftPusherExtended = false;
                    }

                    if (rightPusherExtended)
                    {
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_RETRACT_POSITION);
                        rightPusherExtended = false;
                    }
                    //
                    // It takes sometime for the button pusher to retract, set a timer to wait for it.
                    //
                    timer.set(0.5, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }
        }
    }

}   //class AutoBeacon
