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

public class CmdPushBeaconButtons2 implements TrcRobot.RobotCommand
{
    private enum State
    {
        ALIGN_WALL1,
        ALIGN_WALL2,
        ALIGN_WALL3,
        FIND_LINE,
        PUSH_BUTTON1,
        PUSH_BUTTON2,
        RETRACT,
        NEXT_BEACON,
        GOTO_VORTEX,
        TURN_TO_VORTEX,
        PARK_VORTEX,
        DONE
    }   //enum State

    private static final String moduleName = "CmdPushBeaconButtons2";

    private TrcDbgTrace tracer = FtcOpMode.getGlobalTracer();

    private Robot robot;
    private FtcAuto.Alliance alliance;
    private int beaconButtons;
    private FtcAuto.ParkOption parkOption;
    private int remainingBeaconButtons;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;

    public CmdPushBeaconButtons2(
            Robot robot, FtcAuto.Alliance alliance, int beaconButtons, FtcAuto.ParkOption parkOption)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.beaconButtons = beaconButtons;
        this.parkOption = parkOption;
        remainingBeaconButtons = beaconButtons;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.ALIGN_WALL1);
    }   //CmdPushBeaconButtons2

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = false;
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(1, "State: %s", sm.isReady()? (sm.getState()).toString(): "Disabled");

        if (sm.isReady())
        {
            State state = sm.getState();
            double xDistance, yDistance;
            int redValue, greenValue, blueValue;
            boolean isRed, isBlue;

            robot.traceStateInfo(elapsedTime, state.toString());
            switch (state)
            {
                case ALIGN_WALL1:
                    robot.driveBase.mecanumDrive_Cartesian(-1.0, 0.0, 0.0);
                    timer.set(1.0, event);
                    sm.setState(State.ALIGN_WALL2);
                    break;

                case ALIGN_WALL2:
                    if (robot.getInput(robot.rangePidCtrl) < 3.5 || event.isSignaled())
                    {
                        sm.setState(State.ALIGN_WALL3);
                    }
                    break;

                case ALIGN_WALL3:
                    robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0);

                    if (Robot.USE_COLOR_SENSOR)
                    {
                        robot.beaconColorSensor.setDeviceEnabled(true);
                    }

                    timer.set(0.1, event);
                    sm.waitForSingleEvent(event, State.FIND_LINE);
                    break;

                case FIND_LINE:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE && remainingBeaconButtons == 2 ||
                                alliance == FtcAuto.Alliance.BLUE_ALLIANCE && remainingBeaconButtons == 1? 30.0: -30.0;
                    robot.targetHeading = robot.driveBase.getHeading();

                    if (Robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(true);
                    }

                    robot.encoderYPidCtrl.setOutputRange(-0.12, 0.12);
                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BUTTON1);
                    break;

                case PUSH_BUTTON1:
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    if (Robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(false);
                    }

                    if (Robot.USE_COLOR_SENSOR)
                    {
                        redValue = robot.beaconColorSensor.sensor.red();
                        greenValue = robot.beaconColorSensor.sensor.green();
                        blueValue = robot.beaconColorSensor.sensor.blue();
                    }
                    isRed = redValue > blueValue && redValue > greenValue;
                    isBlue = blueValue > redValue && blueValue > greenValue;
                    robot.dashboard.displayPrintf(
                            14, "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue));
                    tracer.traceInfo(
                            state.toString(), "[%d,%d,%d]isRed=%s,isBlue=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue));
                    //
                    // Determine which button to push and do it.
                    //
                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isRed ||
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isBlue)
                    {
                        //
                        // It takes sometime for the button pusher to extend, set a timer to wait for it.
                        //
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        leftPusherExtended = true;
                        timer.set(1.5, event);
                        sm.waitForSingleEvent(event, State.RETRACT);
                    }
                    else if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isBlue ||
                             alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isRed)
                    {
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        rightPusherExtended = true;
                        timer.set(1.5, event);
                        if (Robot.USE_COLOR_SENSOR)
                        {
                            sm.setState(State.PUSH_BUTTON2);
                        }
                        else
                        {
                            sm.waitForSingleEvent(event, State.RETRACT);
                        }
                    }
                    else
                    {
                        //
                        // Cannot determine the color, skip it.
                        //
                        sm.setState(State.NEXT_BEACON);
                    }
                    robot.dashboard.displayPrintf(
                            15, "leftPusher=%s, rightPusher=%s",
                            Boolean.toString(leftPusherExtended), Boolean.toString(rightPusherExtended));
                    break;

                case PUSH_BUTTON2:
                    //
                    // Keep checking the color of the beacon. If it has changed to our color or time has expired,
                    // move on to the next state.
                    //
                    if (Robot.USE_COLOR_SENSOR)
                    {
                        redValue = robot.beaconColorSensor.sensor.red();
                        greenValue = robot.beaconColorSensor.sensor.green();
                        blueValue = robot.beaconColorSensor.sensor.blue();
                        isRed = redValue > blueValue && redValue > greenValue;
                        isBlue = blueValue > redValue && blueValue > greenValue;
                    }
                    boolean timedOut = event.isSignaled();
                    tracer.traceInfo(
                            state.toString(), "[%d,%d,%d]isRed=%s,isBlue=%s,expired=%s",
                            redValue, greenValue, blueValue, Boolean.toString(isRed), Boolean.toString(isBlue),
                            Boolean.toString(timedOut));
                    if (timedOut ||
                        alliance == FtcAuto.Alliance.RED_ALLIANCE && isRed ||
                        alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isBlue)
                    {
                        robot.beaconColorSensor.setDeviceEnabled(false);
                        sm.setState(State.RETRACT);
                    }
                    break;

                case RETRACT:
                    //
                    // We need to retract the pusher a little bit before start moving so it doesn't get
                    // caught on by the beacon.
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

                    timer.set(0.2, event);
                    sm.waitForSingleEvent(event, State.NEXT_BEACON);
                    break;

                case NEXT_BEACON:
                    if (remainingBeaconButtons == 2)
                    {
                        xDistance = 0.0;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -40.0: 40.0;

                        robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        remainingBeaconButtons--;
                        sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    }
                    else if (parkOption == FtcAuto.ParkOption.DO_NOTHING)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        //
                        // We are going somewhere. let's get off the wall so we can turn.
                        // We don't have enough time to go to the center vortex, so always head for the corner vortex.
                        //
                        xDistance = 36.0;
                        yDistance = 0.0;

                        robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.GOTO_VORTEX);
                    }
                    break;

                case GOTO_VORTEX:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -36.0: 36.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_VORTEX);
                    break;

                case TURN_TO_VORTEX:
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 45.0: 135.0;

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_VORTEX);
                    break;

                case PARK_VORTEX:
                    xDistance = 0.0;
                    if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
                    {
                        yDistance = parkOption == FtcAuto.ParkOption.PARK_CENTER? 36.0: -36.0;
                    }
                    else
                    {
                        yDistance = parkOption == FtcAuto.ParkOption.PARK_CENTER? -36.0: 36.0;
                    }

                    robot.setTurnPID(xDistance, yDistance, robot.targetHeading);
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    done = true;
                    break;
            }
        }

        return done;
    }   //autoPeriodic

}   //class CmdPushBeaconButtons2
