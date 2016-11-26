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
        MOVE_CENTER_VORTEX,
        AIM_CENTER_VORTEX,
        GOTO_CENTER_VORTEX,
        SHOOT_PARTICLES,
        MOVE_OUT,
        TURN_TO_CAPBALL,
        KNOCK_OUT_CAPBALL,
        BACKUP,
        TURN_TO_CENTER1,
        PARK_CENTER1,
        TURN_TO_CORNER1,
        PARK_CORNER1,
        ALIGN_WALL,
        GOTO_WALL,
        BACKOUT,
        FIND_LINE,
        PUSH_BUTTON,
        RETRACT,
        NEXT_BEACON,
        TURN_TO_CENTER2,
        GOTO_CENTER2,
        GOTO_CORNER2,
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
    private FtcAuto.ParkOption parkOption;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine sm;
    private boolean particleLoaded = true;
    private boolean leftPusherExtended = false;
    private boolean rightPusherExtended = false;
    private double startTime = 0.0;
    private double heading = 0.0;
    private int remainingBeaconButtons = 0;

    public AutoBeacon(
            Robot robot,
            FtcAuto.Alliance alliance,
            FtcAuto.StartPosition startPos,
            double delay,
            int shootParticles,
            int beaconButtons,
            FtcAuto.ParkOption parkOption)
    {
        this.robot = robot;
        this.alliance = alliance;
        this.startPos = startPos;
        this.delay = delay;
        this.shootParticles = shootParticles;
        this.beaconButtons = beaconButtons;
        this.remainingBeaconButtons = beaconButtons;
        this.parkOption = parkOption;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine(moduleName);
        sm.start(startPos == FtcAuto.StartPosition.NEAR? State.SHOOT_PARTICLES: State.MOVE_CENTER_VORTEX);
    }

    @Override
    public void autoPeriodic(double elapsedTime)
    {
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
            State nextState;
            double driveDistance;

            traceStateInfo(elapsedTime, state);
            switch (state)
            {
                case MOVE_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 6.0: 8.0;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.AIM_CENTER_VORTEX);
                    break;

                case AIM_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    heading = selectParameter(startPos, alliance, 0.0, 0.0, -45.0, 43.0);

                    robot.pidDrive.setTarget(0.0, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER_VORTEX);
                    break;

                case GOTO_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    driveDistance = 6.0;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.SHOOT_PARTICLES);
                    break;

                case SHOOT_PARTICLES:
                    tracer.traceInfo(state.toString(), "NumParticles=%d", shootParticles);
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
                        sm.waitForSingleEvent(event, State.SHOOT_PARTICLES);
                    }
                    //
                    // Do delay if any.
                    //
                    else
                    {
                        nextState = startPos == FtcAuto.StartPosition.NEAR? State.MOVE_OUT: State.TURN_TO_CAPBALL;
                        if (delay > 0.0 && delay - elapsedTime > 0)
                        {
                            timer.set(delay - elapsedTime, event);
                            sm.waitForSingleEvent(event, nextState);
                        }
                        else
                        {
                            sm.setState(nextState);
                        }
                    }
                    break;

                case MOVE_OUT:
                    driveDistance = 12.0;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CAPBALL);
                    break;

                case TURN_TO_CAPBALL:
                    heading = selectParameter(startPos, alliance, -13.0, 18.0, -58.0, 59.0);

                    robot.pidDrive.setTarget(0.0, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.KNOCK_OUT_CAPBALL);
                    break;

                case KNOCK_OUT_CAPBALL:
                    nextState = beaconButtons == 0? State.BACKUP: State.ALIGN_WALL;

                    driveDistance = selectParameter(
                            startPos, alliance, 60.0, beaconButtons != 0? 72.0: 60.0, 80.0, 80.0);

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACKUP:
                    if (parkOption == FtcAuto.ParkOption.PARK_CORNER)
                    {
                        driveDistance = selectParameter(startPos, alliance, -30.0, -25.0, -20.0, -20.0);
                    }
                    else
                    {
                        driveDistance = selectParameter(startPos, alliance, -64.0, -66.0, -46.0, -50.0);
                    }
                    nextState = parkOption == FtcAuto.ParkOption.PARK_CENTER? State.TURN_TO_CENTER1:
                                parkOption == FtcAuto.ParkOption.PARK_CORNER? State.TURN_TO_CORNER1: State.DONE;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_CENTER1:
                    heading = selectParameter(startPos, alliance, 13.0, -11.0, 0.0, -6.0);

                    robot.pidDrive.setTarget(0.0, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CENTER1);
                    break;

                case PARK_CENTER1:
                    driveDistance = selectParameter(startPos, alliance, 50.0, 50.0, 22.0, 35.0);

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_CORNER1:
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -135.0: 135.0;

                    robot.pidDrive.setTarget(0.0, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CORNER1);
                    break;

                case PARK_CORNER1:
                    driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 60.0: 56.0;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case ALIGN_WALL:
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0;

                    robot.pidDrive.setTarget(0.0, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    //
                    // Crab towards wall using the Range Sensor.
                    //
                    driveDistance = 1.5;

                    robot.rangePidDrive.setTarget(driveDistance, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.BACKOUT);
                    /*
                    if (beaconButtons == 2 && remainingBeaconButtons == 1)
                    {
                        driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -7.0: -7.0;
                    }
                    else
                    {
                        driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -37.0: -30.0;
                    }

                    robot.pidDrive.setTarget(driveDistance, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.BACKOUT);
                    */
                    break;

                case BACKOUT:
                    //
                    // The GOTO_WALL state will correct the robot alignment, remember this new heading.
                    //
                    heading = robot.driveBase.getHeading();
                    driveDistance = 3.0;

                    robot.rangePidDrive.setTarget(driveDistance, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.FIND_LINE);
                    break;

                case FIND_LINE:
                    robot.lineTrigger.setEnabled(true);
                    robot.encoderYPidCtrl.setOutputRange(-0.1, 0.1);

                    driveDistance = -30.0;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BUTTON);
                    break;

                case PUSH_BUTTON:
                    robot.lineTrigger.setEnabled(false);
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);

                    int redValue = robot.beaconColorSensor.red();
                    int greenValue = robot.beaconColorSensor.green();
                    int blueValue = robot.beaconColorSensor.blue();
                    boolean isRed = redValue > blueValue && redValue > greenValue;
                    boolean isBlue = blueValue > redValue && blueValue > greenValue;
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
                        robot.leftButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        leftPusherExtended = true;
                    }
                    else if (alliance == FtcAuto.Alliance.RED_ALLIANCE && isBlue ||
                             alliance == FtcAuto.Alliance.BLUE_ALLIANCE && isRed)
                    {
                        robot.rightButtonPusher.setPosition(RobotInfo.BUTTON_PUSHER_EXTEND_POSITION);
                        rightPusherExtended = true;
                    }
                    robot.dashboard.displayPrintf(
                            15, "leftPusher=%s, rightPusher=%s",
                            Boolean.toString(leftPusherExtended), Boolean.toString(rightPusherExtended));
                    //
                    // It takes sometime for the button pusher to extend, set a timer to wait for it.
                    //
                    if (leftPusherExtended || rightPusherExtended)
                    {
                        timer.set(1.0, event);
                        sm.waitForSingleEvent(event, State.RETRACT);
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
                    timer.set(0.2, event);
                    sm.waitForSingleEvent(event, State.NEXT_BEACON);
                    break;

                case NEXT_BEACON:
                    if (remainingBeaconButtons == 2)
                    {
                        driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 57.0: 57.0;

                        robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                        remainingBeaconButtons--;
                        sm.waitForSingleEvent(event, State.GOTO_WALL);
                    }
                    else if (parkOption == FtcAuto.ParkOption.DO_NOTHING)
                    {
                        //
                        // Stay there, we are done!
                        //
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        //
                        // We are going somewhere. let's get off the wall so we can turn.
                        //
                        nextState = State.GOTO_CORNER2;
//                        nextState = parkOption == FtcAuto.ParkOption.PARK_CENTER?
//                                State.TURN_TO_CENTER2: State.GOTO_CORNER2;

                        driveDistance = 12.0;

                        robot.pidDrive.setTarget(driveDistance, 0.0, heading, false, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;

                case TURN_TO_CENTER2:
                    if (beaconButtons == 2)
                    {
                        heading = 135.0;
                    }
                    else
                    {
                        heading = 45.0;
                    }

                    robot.pidDrive.setTarget(0.0, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER2);
                    break;

                case GOTO_CENTER2:
                    driveDistance = beaconButtons == 2? 60.0: 48.0;

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case GOTO_CORNER2:
                    if (beaconButtons == 2)
                    {
                        driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -84.0: 48.0;
                    }
                    else
                    {
                        driveDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -40.0: 84.0;
                    }

                    robot.pidDrive.setTarget(0.0, driveDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
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
    }   //autoPeriodic

    private double selectParameter(
            FtcAuto.StartPosition startPos, FtcAuto.Alliance alliance,
            double nearRed, double nearBlue, double farRed, double farBlue)
    {
        if (startPos == FtcAuto.StartPosition.NEAR)
        {
            if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
            {
                return nearRed;
            }
            else
            {
                return nearBlue;
            }
        }
        else
        {
            if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
            {
                return farRed;
            }
            else
            {
                return farBlue;
            }
        }
    }   //selectParameter

    private void traceStateInfo(double elapsedTime, State state)
    {
        tracer.traceInfo(
                state.toString(), "[%5.3f] ********** StateElapsedTime=%5.3f ***********",
                elapsedTime, startTime == 0? 0.0: elapsedTime - startTime);
        startTime = elapsedTime;
        robot.encoderXPidCtrl.printPidInfo(tracer);
        robot.encoderYPidCtrl.printPidInfo(tracer);
        robot.gyroPidCtrl.printPidInfo(tracer);
    }   //traceStateInfo

}   //class AutoBeacon
