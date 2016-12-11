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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        TURN_TO_BEACON,
        GOTO_WALL,
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
        if (robot.USE_LINE_DETECTOR)
        {
            if (robot.USE_ODS_LINE_DETECTOR)
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%.3f",
                                              (Double)robot.odsLineDetector.getRawData(
                                                      0, TrcAnalogInput.DataType.INPUT_DATA).value);
            }
            else
            {
                robot.dashboard.displayPrintf(2, "LineDetect: light=%d",
                                              (Integer)robot.lineDetectionSensor.getWhiteValue().value);
            }
        }

        if (robot.USE_COLOR_SENSOR)
        {
            robot.dashboard.displayPrintf(3, "RGBAH: [%d,%d,%d,%d,%x]",
                                          robot.beaconColorSensor.red(), robot.beaconColorSensor.green(),
                                          robot.beaconColorSensor.blue(), robot.beaconColorSensor.alpha(),
                                          robot.beaconColorSensor.argb());
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

            traceStateInfo(elapsedTime, state);
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
                    yDistance = 12.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CAPBALL);
                    break;

                case TURN_TO_CAPBALL:
                    xDistance = yDistance = 0.0;
                    heading = selectParameter(
                            startPos, alliance,
                            //NEAR
                            -13.0, 15.0,
                            //FAR
                            -54.0, 50.0);

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.KNOCK_OUT_CAPBALL);
                    break;

                case KNOCK_OUT_CAPBALL:
                    nextState = beaconButtons == 0? State.BACKUP: State.TURN_TO_BEACON;
                    xDistance = 0.0;
                    yDistance = selectParameter(
                            startPos, alliance,
                            //NEAR
                            beaconButtons != 0? 64.0: 60, beaconButtons != 0? 70.0: 52.0,
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
                        yDistance = selectParameter(
                                startPos, alliance,
                                //NEAR
                                -24.0, -17.0,
                                //FAR
                                -24.0, -20.0);
                    }
                    else
                    {
                        yDistance = selectParameter(
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
                    heading = selectParameter(
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
                    yDistance = selectParameter(
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

                case TURN_TO_BEACON:
                    xDistance = yDistance = 0.0;
                    heading = alliance == FtcAuto.Alliance.RED_ALLIANCE? 0.0: 180.0;

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    //
                    // Crab towards wall using the Range Sensor.
                    //
                    if (robot.USE_RANGE_SENSOR)
                    {
                        xDistance = 6.0;
                        yDistance = 0.0;
                    }
                    else
                    {
                        if (beaconButtons == 2 && remainingBeaconButtons == 1)
                        {
                            xDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -7.0: -7.0;
                        }
                        else
                        {
                            xDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -37.0: -25.0;
                        }
                        yDistance = 0.0;
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.rangePidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_WALL1);
                    break;

                case ALIGN_WALL1:
                    robot.driveBase.mecanumDrive_Cartesian(-0.75, 0.0, 0.0);
                    timeout = elapsedTime + 1.5;
                    sm.setState(State.ALIGN_WALL2);
                    break;

                case ALIGN_WALL2:
                    if (robot.rangeSensor.sensor.getDistance(DistanceUnit.INCH) < 3.5 || elapsedTime >= timeout)
                    {
                        robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0);
                        sm.setState(State.FIND_LINE);
                    }
                    break;

                case FIND_LINE:
                    xDistance = 0.0;
                    yDistance = -24.0;
                    heading = robot.driveBase.getHeading();

                    if (robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(true);
                    }

//                    robot.encoderYPidCtrl.setDebugTraceEnabled(tracer, true);
                    robot.encoderYPidCtrl.setOutputRange(-0.12, 0.12);
                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.PUSH_BUTTON);
                    break;

                case PUSH_BUTTON:
//                    robot.encoderYPidCtrl.setDebugTraceEnabled(null, false);
                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    if (robot.USE_LINE_DETECTOR)
                    {
                        robot.lineTrigger.setEnabled(false);
                    }

                    int redValue = 0;
                    int greenValue = 0;
                    int blueValue = 0;
                    if (robot.USE_COLOR_SENSOR)
                    {
                        redValue = robot.beaconColorSensor.red();
                        greenValue = robot.beaconColorSensor.green();
                        blueValue = robot.beaconColorSensor.blue();
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
                    //
                    // It takes sometime for the button pusher to extend, set a timer to wait for it.
                    //
                    if (leftPusherExtended || rightPusherExtended)
                    {
                        timer.set(1.5, event);
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
                        xDistance = 0.0;
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 57.0: 57.0;

                        robot.setTurnPID(xDistance, yDistance, heading);
                        robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
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
                        // We don't have enough time to go to the center vortex, so always head for the corner vortex.
                        //
                        nextState = alliance == FtcAuto.Alliance.RED_ALLIANCE? State.GOTO_CORNER2: State.TURN_TO_CORNER;

                        xDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? 24.0: 36.0;
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
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -72.0: 20.0;
                    }
                    else
                    {
                        yDistance = alliance == FtcAuto.Alliance.RED_ALLIANCE? -40.0: 84.0;
                    }

                    robot.setTurnPID(xDistance, yDistance, heading);
                    robot.pidDrive.setTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_CORNER);
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
                moduleName, "[%5.3f] %17s: xPos=%6.2f,yPos=%6.2f,heading=%6.1f/%6.1f,range=%5.2f,volt=%5.2fV(%5.2fV)",
                elapsedTime, state.toString(),
                robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading(), heading,
                robot.rangeSensor.sensor.getDistance(DistanceUnit.INCH),
                robot.battery.getCurrentVoltage(), robot.battery.getLowestVoltage());
    }   //traceStateInfo

}   //class AutoBeacon
