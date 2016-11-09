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
        GOTO_BEACONS,
        ALIGN_WALL,
        GOTO_WALL,
        GOTO_FAR_BEACON,
        GOTO_NEAR_BEACON,
        PUSH_BUTTON,
        RETRACT,
        NEXT_BEACON_BUTTON,
        MOVE_SOMEWHERE,
        GO_DEFENSE,
        PARK_CORNER,
        PARK_CENTER,
        BACK_TO_MOUNTAIN,
        TURN_TO_MOUNTAIN,
        GO_UP_MOUNTAIN,
        DONE
    }   //enum State

    private static final String moduleName = "AutoBeacon";

    private HalDashboard dashboard;
    private TrcDbgTrace tracer = FtcOpMode.getOpModeTracer();

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
                        robot.shooter.fireOneShot(event);
                        shootParticles--;
                        sm.addEvent(event);
                        sm.waitForEvents(State.SHOOT_PARTICLES);
                    }
                    //
                    // Do delay if any.
                    //
                    else if (delay > 0.0)
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(State.GOTO_BEACONS);
                    }
                    else
                    {
                        sm.setState(State.GOTO_BEACONS);
                    }
                    break;

                case GOTO_BEACONS:
                    //
                    // If we are pressing any beacon buttons, go towards the middle of the two beacons.
                    //
                    if (beaconButtons != FtcAuto.BeaconButtons.NONE)
                    {
                        robot.pidDrive.setTarget(0.0, 60.0, 0.0, false, event, 0.0);
                        sm.addEvent(event);
                        sm.waitForEvents(State.ALIGN_WALL);
                    }
                    else
                    {
//                        sm.setState(State.); //???
                    }
                    break;

                case ALIGN_WALL:
                    //
                    // Turn to parallel the wall.
                    //
                    robot.pidDrive.setTarget(
                            0.0, 0.0, alliance == FtcAuto.Alliance.RED_ALLIANCE? 45.0: -45.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.GOTO_WALL);
                    break;

                case GOTO_WALL:
                    //
                    // Crab towards the wall.
                    //
                    robot.pidDrive.setTarget(
                            alliance == FtcAuto.Alliance.RED_ALLIANCE? -30.0: 30.0, 0.0, 0.0, false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.GOTO_FAR_BEACON);
                    break;

                case GOTO_FAR_BEACON:
                    if (beaconButtons == FtcAuto.BeaconButtons.NEAR_BEACON)
                    {
                        //
                        // Skipping the far beacon.
                        //
                        sm.setState(State.GOTO_NEAR_BEACON);
                    }
                    else
                    {
                        //
                        // Go towards the far beacon.
                        //
                        robot.lineTrigger.setEnabled(true);
                        robot.pidDrive.setTarget(
                                0.0, alliance == FtcAuto.Alliance.RED_ALLIANCE? -24.0: 24.0, 0.0, false, event, 0.0);
                        sm.addEvent(event);
                        sm.waitForEvents(State.PUSH_BUTTON);
                    }
                    break;

                case GOTO_NEAR_BEACON:
                    //
                    // Go towards the near beacon.
                    //
                    robot.lineTrigger.setEnabled(true);
                    double dir = alliance == FtcAuto.Alliance.RED_ALLIANCE? 1.0: -1.0;
                    robot.pidDrive.setTarget(
                            0.0, beaconButtons == FtcAuto.BeaconButtons.NEAR_BEACON? dir*24.0: dir*48.0, 0.0,
                            false, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.PUSH_BUTTON);
                    break;

                case PUSH_BUTTON:
                    //
                    // Determine which button to press and press it.
                    //
                    robot.lineTrigger.setEnabled(false);
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
                    timer.set(3.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.RETRACT);
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
                    sm.waitForEvents(State.NEXT_BEACON_BUTTON);
                    break;

                case NEXT_BEACON_BUTTON:
                    if (beaconButtons == FtcAuto.BeaconButtons.BOTH)
                    {
                        beaconButtons = FtcAuto.BeaconButtons.NEAR_BEACON;
                        sm.setState(State.GOTO_NEAR_BEACON);
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
                        //
                        // We are going to move out of the way.
                        // First we need to back up a little bit so we have some room to turn.
                        //
                        robot.pidDrive.setTarget(
                                -6.0, 0.0, false, event, 1.0);
                        sm.addEvent(event);
                        sm.waitForEvents(State.MOVE_SOMEWHERE);
                    }
                    break;

                case MOVE_SOMEWHERE:
                    if (option == FtcAuto.BeaconOption.DEFENSE)
                    {
                        //
                        // We can only go to the opponent's side after 10 seconds.
                        // Check it just to be safe.
                        //
                        if (elapsedTime > 10.0)
                        {
                            sm.setState(State.GO_DEFENSE);
                        }
                    }
                    else if (option == FtcAuto.BeaconOption.PARK_CENTER)
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
                        sm.waitForEvents(State.BACK_TO_MOUNTAIN);
                    }
                    break;

                case GO_DEFENSE:
                    //
                    // Run to the opponent side and bump them if necessary.
                    //
                    robot.pidDrive.setTarget(
                            -35.0, alliance == FtcAuto.Alliance.RED_ALLIANCE? -45.0: 45.0,
                            false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case PARK_CORNER:
                    //
                    // Go into the floor goal.
                    //
                    robot.pidDrive.setTarget(-24.0, 0.0, false, event, 3.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case BACK_TO_MOUNTAIN:
                    //
                    // Back up to mountain foothill.
                    //
                    robot.pidDrive.setTarget(-42.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.TURN_TO_MOUNTAIN);
                    break;

                case TURN_TO_MOUNTAIN:
                    //
                    // Turn to face the mountain.
                    //
                    robot.pidDrive.setTarget(
                            0.0, alliance == FtcAuto.Alliance.RED_ALLIANCE? -90.0: 90.0,
                            false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.GO_UP_MOUNTAIN);
                    break;

                case GO_UP_MOUNTAIN:
                    robot.pidDrive.setTarget(50.0, 0.0, false, event, 5.0);
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
