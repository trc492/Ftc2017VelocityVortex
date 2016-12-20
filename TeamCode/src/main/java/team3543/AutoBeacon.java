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
import trclib.TrcAnalogInput;
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
        TURN_TO_WALL,
        GOTO_WALL,
        PARALLEL_WALL,
        ALIGN_WALL1,
        ALIGN_WALL2,
        FIND_LINE,
        PUSH_BUTTON,
        RETRACT,
        NEXT_BEACON,
        TURN_TO_CENTER2,
        GOTO_CENTER2,
        GOTO_CORNER2,
        TURN_TO_CORNER,
        GO_UP_CORNER,
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
    private double heading = 0.0;
    private int remainingBeaconButtons = 0;
    private double timeout = 0.0;

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
        robot.battery.setTaskEnabled(true);
        sm.start(startPos == FtcAuto.StartPosition.NEAR? State.SHOOT_PARTICLES: State.MOVE_CENTER_VORTEX);
        tracer.traceInfo(moduleName, "********** Starting autonomous **********");
    }

    @Override
    public void autoPeriodic(double elapsedTime)
    {
        //
        // Print debug info.
        //
        robot.dashboard.displayPrintf(
                1, "State: %s", sm.isReady()? ((State)sm.getState()).toString(): "Disabled");
        if (Robot.USE_LINE_DETECTOR)
        {
            if (Robot.USE_ODS_LINE_DETECTOR)
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%.3f",
                                              (Double)robot.odsLineDetector.getRawData(
                                                      0, TrcAnalogInput.DataType.INPUT_DATA).value);
            }
            else
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%d", robot.lineDetectionSensor.sensor.alpha());
            }
        }

        if (Robot.USE_COLOR_SENSOR)
        {
            robot.dashboard.displayPrintf(3, "RGBAH: [%d,%d,%d,%d,%x]",
                                          robot.beaconColorSensor.sensor.red(), robot.beaconColorSensor.sensor.green(),
                                          robot.beaconColorSensor.sensor.blue(), robot.beaconColorSensor.sensor.alpha(),
                                          robot.beaconColorSensor.sensor.argb());
        }

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
            double xDistance;
            double yDistance;
            int redValue = 0;
            int greenValue = 0;
            int blueValue = 0;

            robot.traceStateInfo(elapsedTime, state.toString(), heading);
            switch (state)
            {
                case MOVE_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 6.0: 8.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.AIM_CENTER_VORTEX);
                    break;

                case AIM_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -45.0: 43.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER_VORTEX);
                    break;

                case GOTO_CENTER_VORTEX:
                    //
                    // This state is called only for FAR StartPosition.
                    //
                    xDistance = 0.0;
                    yDistance = 6.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
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
                    xDistance = 0.0;
                    yDistance = 32.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CAPBALL);
                    break;

                case TURN_TO_CAPBALL:
                    xDistance = yDistance = 0.0;
                    heading = robot.selectParameter(
                            startPos, alliance,
                            //NEAR
                            -30.0, 30.0,
                            //FAR
                            -54.0, 50.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.KNOCK_OUT_CAPBALL);
                    break;

                case KNOCK_OUT_CAPBALL:
                    nextState = beaconButtons == 0? State.BACKUP: State.TURN_TO_WALL;
                    xDistance = 0.0;
                    yDistance = robot.selectParameter(
                            startPos, alliance,
                            //NEAR (old value = 64.0)
                            beaconButtons != 0? 45.0: 45, beaconButtons != 0? 70.0: 52.0,
                            //FAR
                            80.0, 80.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACKUP:
                    nextState = parkOption == FtcAuto.ParkOption.PARK_CENTER? State.TURN_TO_CENTER1:
                                parkOption == FtcAuto.ParkOption.PARK_CORNER? State.TURN_TO_CORNER1: State.DONE;
                    xDistance = 0.0;
                    if (parkOption == FtcAuto.ParkOption.PARK_CORNER)
                    {
                        yDistance = robot.selectParameter(
                                startPos, alliance,
                                //NEAR
                                -24.0, -17.0,
                                //FAR
                                -24.0, -20.0);
                    }
                    else
                    {
                        yDistance = robot.selectParameter(
                                startPos, alliance,
                                //NEAR
                                -64.0, -58.0,
                                //FAR
                                -46.0, -50.0);
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TO_CENTER1:
                    xDistance = yDistance = 0.0;
                    heading = robot.selectParameter(
                            startPos, alliance,
                            //NEAR
                            13.0, -11.0,
                            //FAR
                            -0.3, -2.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CENTER1);
                    break;

                case PARK_CENTER1:
                    xDistance = 0.0;
                    yDistance = robot.selectParameter(
                            startPos, alliance,
                            //NEAR
                            50.0, 50.0,
                            //FAR
                            22.0, 32.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_CORNER1:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -130.0: 135.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARK_CORNER1);
                    break;

                case PARK_CORNER1:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 52.0: 52.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_WALL:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? -90.0: 90.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    xDistance = 0.0;
                    yDistance = 22.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PARALLEL_WALL);
                    break;

                case PARALLEL_WALL:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 10.0: 180.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    break;

                case ALIGN_WALL1:
                    if (Robot.USE_RANGE_SENSOR)
                    {
                        robot.rangeSensor.setDeviceEnabled(true);
                    }
                    robot.driveBase.mecanumDrive_Cartesian(-1.0, 0.0, 0.0);
                    timeout = elapsedTime + 1.0;
                    sm.setState(State.ALIGN_WALL2);
                    break;

                case ALIGN_WALL2:
                    if (robot.getInput(robot.rangePidCtrl) < 3.5 || elapsedTime >= timeout)
                    {
                        robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0);
                        sm.setState(State.FIND_LINE);
                        if (Robot.USE_RANGE_SENSOR)
                        {
                            robot.rangeSensor.setDeviceEnabled(false);
                        }

                        if (Robot.USE_COLOR_SENSOR)
                        {
                            robot.beaconColorSensor.setDeviceEnabled(true);
                        }
                    }
                    break;

                case FIND_LINE:
                    xDistance = 0.0;
                    yDistance = -24.0;
                    heading = robot.driveBase.getHeading();

                    if (Robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(true);
                    }

                    robot.encoderYPidCtrl.setOutputRange(-0.12, 0.12);
                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BUTTON);
                    break;

                case PUSH_BUTTON:
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
                    if (leftPusherExtended || rightPusherExtended)
                    {
                        timer.set(1.5, event);
                        sm.setState(State.RETRACT);
                    }
                    else
                    {
                        sm.setState(State.NEXT_BEACON);
                    }
                    break;

                case RETRACT:
                    //
                    // It takes sometime for the button pusher to extend, either wait for the color change or
                    // a timeout.
                    //
                    if (Robot.USE_COLOR_SENSOR)
                    {
                        int currRed = robot.beaconColorSensor.sensor.red();
                        int currGreen = robot.beaconColorSensor.sensor.green();
                        int currBlue = robot.beaconColorSensor.sensor.blue();
                        tracer.traceInfo(
                                state.toString(), "[%d,%d,%d]->[%d,%d,%d] (expired=%s)",
                                redValue, greenValue, blueValue, currRed, currGreen, currBlue,
                                Boolean.toString(event.isSignaled()));
                        //
                        // Either the beacon has changed color or we timed out, start retracting the button pusher
                        // and move on.
                        //
                        if (currRed > redValue || currBlue > blueValue || event.isSignaled())
                        {
                            robot.beaconColorSensor.setDeviceEnabled(false);
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
                            // We need to retract the pusher a little bit before start moving so it doesn't get
                            // caught on by the beacon.
                            //
                            timer.set(0.2, event);
                            sm.waitForSingleEvent(event, State.NEXT_BEACON);
                        }
                    }
                    else
                    {
                        //
                        // Should never come here but just in case.
                        //
                        sm.setState(State.NEXT_BEACON);
                    }
                    break;

                case NEXT_BEACON:
                    if (remainingBeaconButtons == 2)
                    {
                        xDistance = 0.0;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 57.0: 57.0;

                        robot.setTurnPID(xDistance, yDistance, heading);
                        robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                        remainingBeaconButtons--;
                        sm.waitForSingleEvent(event, State.ALIGN_WALL1);
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
                        // We don't have enough time to go to the center vortex, so always head for the corner vortex.
                        //
                        nextState = alliance == FtcAuto.Alliance.RED_ALLIANCE? State.GOTO_CORNER2: State.TURN_TO_CORNER;

                        xDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 12.0: 24.0;
                        yDistance = 0.0;

                        robot.setTurnPID(xDistance, yDistance, heading);
                        robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;

                case TURN_TO_CENTER2:
                    xDistance = yDistance = 0.0;
                    if (beaconButtons == 2)
                    {
                        heading = 135.0;
                    }
                    else
                    {
                        heading = 45.0;
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_CENTER2);
                    break;

                case GOTO_CENTER2:
                    xDistance = 0.0;
                    yDistance = beaconButtons == 2? 60.0: 48.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case GOTO_CORNER2:
                    xDistance = 0.0;
                    if (beaconButtons == 2)
                    {
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -96.0: 20.0;
                    }
                    else
                    {
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -40.0: 84.0;
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case TURN_TO_CORNER:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 45.0: 135.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GO_UP_CORNER);
                    break;

                case GO_UP_CORNER:
                    xDistance = 0.0;
                    yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -48.0: 48.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.battery.setTaskEnabled(false);
                    sm.stop();
                    break;
            }
        }
    }   //autoPeriodic

}   //class AutoBeacon
