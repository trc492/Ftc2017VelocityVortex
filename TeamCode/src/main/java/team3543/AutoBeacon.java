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
        NEAR_GOTO_BEACON,
        FAR_GOTO_BEACON,
        ALIGN_WALL,
        GOTO_WALL,
        FIND_LINE,
        PUSH_BUTTON,
        RETRACT,
        NEXT_BEACON,
        KNOCK_OUT_CAPBALL,
        DONE
    }   //enum State

    private static final String moduleName = "AutoBeacon";

    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private FtcAuto.StartPosition startPos;
    private double delay;
    private int shootParticles;
    private int beaconButtons;
    private FtcAuto.BeaconOption option;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    private boolean particleLoaded = true;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;

    public AutoBeacon(
            Robot robot,
            FtcAuto.Alliance alliance,
            FtcAuto.StartPosition startPos,
            double delay,
            int shootParticles,
            int beaconButtons,
            FtcAuto.BeaconOption option)
    {
        this.robot = robot;
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
        State nextState;
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(
                1, "State: %s", sm.isReady()? ((State)sm.getState()).toString(): "Disabled");
        robot.dashboard.displayPrintf(
                2, "LineDetect: light=%d",
                (Integer)robot.lineDetectionSensor.getWhiteValue().value);
        robot.dashboard.displayPrintf(
                3, "RGBAH: [%d,%d,%d,%d,%x]",
                robot.beaconColorSensor.red(),
                robot.beaconColorSensor.green(),
                robot.beaconColorSensor.blue(),
                robot.beaconColorSensor.alpha(),
                robot.beaconColorSensor.argb());
        if (robot.pidDrive.isEnabled())
        {
            robot.encoderXPidCtrl.displayPidInfo(4);
            robot.encoderYPidCtrl.displayPidInfo(6);
            robot.gyroPidCtrl.displayPidInfo(8);
        }

        if (sm.isReady())
        {
            State state = (State)sm.getState();

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
                    else
                    {
                        nextState = beaconButtons == 0?
                                            State.KNOCK_OUT_CAPBALL:
                                    startPos == FtcAuto.StartPosition.NEAR?
                                            State.NEAR_GOTO_BEACON: State.FAR_GOTO_BEACON;
                        //
                        // Do delay if any.
                        //
                        if (delay > 0.0 && delay - elapsedTime > 0)
                        {
                            timer.set(delay - elapsedTime, event);
                            sm.addEvent(event);
                            sm.waitForEvents(nextState);
                        }
                        else
                        {
                            sm.setState(nextState);
                        }
                    }
                    break;

                case NEAR_GOTO_BEACON:
                    robot.pidDrive.setTarget(0.0, 60.0, -45.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.ALIGN_WALL);
                    break;

                case FAR_GOTO_BEACON:
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
                        sm.setState(State.NEXT_BEACON);
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
                    sm.waitForEvents(State.NEXT_BEACON);
                    break;

                case NEXT_BEACON:
                    if (beaconButtons == 2)
                    {
                        robot.pidDrive.setTarget(0.0, 55.0, 0.0, false, event, 0.0);
                        beaconButtons--;
                        sm.addEvent(event);
                        sm.waitForEvents(State.FIND_LINE);
                    }
                    else if (option == FtcAuto.BeaconOption.DO_NOTHING)
                    {
                        //
                        // Stay there, we are done!
                        //
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);    //????
                    }
                    break;

                case KNOCK_OUT_CAPBALL:
                    robot.pidDrive.setTarget(0.0, 60.0, 0.0, false, event, 0.0);
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

                /*
                case BACKUP:
                    //
                    // We are going to move out of the way.
                    // First we need to back up a little bit so we have some room to turn.
                    //
                    robot.pidDrive.setTarget(12.0, 0.0, 0.0, false, event, 1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.MOVE_SOMEWHERE);
                    break;

                case MOVE_SOMEWHERE:
                    if (option == FtcAuto.BeaconOption.PARK_CENTER)
                    {
                        //
                        // Turn to face the floor goal.
                        //
                        robot.pidDrive.setTarget(
                                0.0, alliance == FtcAuto.Alliance.RED_ALLIANCE? 90.0: -90.0,
                                false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.PARK_CENTER);
                    }
                    else if (option == FtcAuto.BeaconOption.PARK_CORNER)
                    {
                        //
                        // Turn to parallel the mountain.
                        //
                        robot.pidDrive.setTarget(
                                0.0, alliance == FtcAuto.Alliance.RED_ALLIANCE? 45.0: -45.0,
                                false, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.PARK_CORNER);
                    }
                    break;

                case PARK_CORNER:
                    //
                    // Go into the floor goal.
                    //
                    robot.pidDrive.setTarget(-24.0, 0.0, false, event, 3.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case PARK_CENTER:
                    //
                    // Back up to mountain foothill.
                    //
                    robot.pidDrive.setTarget(-42.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;
                */
            }
        }
    }

}   //class AutoBeacon
