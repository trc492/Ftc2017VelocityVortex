package team3543;

import ftclib.FtcDcMotor;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcServo;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

/**
 * Created by jzheng on 11/7/2016.
 */

public class SimpleShooter implements TrcTaskMgr.Task, TrcPidController.PidInput {
    private enum SimpleShooterState
    {
        PULL_BACK,
        OPEN_BALL_GATE,
        CLOSE_BALL_GATE,
        FIRE,
        DONE
    }   //enum State

    private String instanceName;
    private FtcDcMotor shooterMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private TrcStateMachine sm;
    private TrcTimer timer;
    private TrcEvent event;
    private TrcServo gateServo;
    private boolean continuousModeOn = false;
    private double pullBackDistance = RobotInfo.SHOOTER_SIMPLE_PULLBACK_TARGET;
    private double oneShotDistance = RobotInfo.SHOOTER_SIMPLE_ONESHOT_TARGET;
    private double openBallGateDelay  = RobotInfo.SHOOTER_SIMPLE_OPEN_GATE_DELAY;
    private double closeBallGateDelay  = RobotInfo.SHOOTER_SIPMLE_CLOSE_GATE_DELAY;

    public SimpleShooter(String instanceName, TrcServo servo)
    {
        this.instanceName = instanceName;

        shooterMotor = new FtcDcMotor("shooterMotor");
        shooterMotor.setInverted(true);
        shooterMotor.setBrakeModeEnabled(true);
        shooterMotor.setSpeedTaskEnabled(true);
        pidCtrl = new TrcPidController(
                instanceName,
                RobotInfo.SHOOTER_KP, RobotInfo.SHOOTER_KI, RobotInfo.SHOOTER_KD, RobotInfo.SHOOTER_KF,
                RobotInfo.SHOOTER_TOLERANCE, RobotInfo.SHOOTER_SETTLING, this);
        pidMotor = new TrcPidMotor(instanceName, shooterMotor, pidCtrl);

        sm = new TrcStateMachine(instanceName);
        event = new TrcEvent(instanceName);
        timer = new TrcTimer(instanceName);
        gateServo = servo;
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    public void stop()
    {
        if (sm.isEnabled())
        {
            timer.cancel();
            sm.stop();
        }
        setEnabled(false);
        continuousModeOn = false;
        shooterMotor.setPower(0.0);
        gateServo.setPosition(RobotInfo.BALLGATE_CLOSE_POSITION);
    }

    public double getSpeed()
    {
        return shooterMotor.getSpeed();
    }

    public double getPosition()
    {
        return shooterMotor.getPosition();
    }

    private void fire(boolean continuous)
    {
        continuousModeOn = continuous;
        if (!sm.isEnabled())
        {
            sm.start(SimpleShooterState.PULL_BACK);
            setEnabled(true);
        }
    }

    public void fireOneShot()
    {
        fire(false);
    }

    public void fireContinuous(boolean on)
    {
        continuousModeOn = on;
        if (on)
        {
            fire(true);
        }
    }

    public void setShooterParameters(double pullbackDist, double oneShotDist, double openGateDelay, double closeGateDelay) {
        pullBackDistance = pullbackDist;
        oneShotDistance = oneShotDist;
        openBallGateDelay = openGateDelay;
        closeBallGateDelay = closeGateDelay;
    }

    public void setPowerManually(double power) {
        shooterMotor.setPower(power);
    }
    //
    // Implements TrcTaskMgr.Task.
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        if (sm.isReady())
        {
            SimpleShooterState state = (SimpleShooterState) sm.getState();
            switch (state)
            {
                case PULL_BACK:
                    pidMotor.setTarget(pullBackDistance, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(SimpleShooterState.OPEN_BALL_GATE);
                    break;

                case OPEN_BALL_GATE:
                    gateServo.setPosition(RobotInfo.BALLGATE_OPEN_POSITION);
                    timer.set(openBallGateDelay, event);
                    sm.addEvent(event);
                    sm.waitForEvents(SimpleShooterState.CLOSE_BALL_GATE);
                    break;

                case CLOSE_BALL_GATE:
                    gateServo.setPosition(RobotInfo.BALLGATE_CLOSE_POSITION);
                    timer.set(closeBallGateDelay, event);
                    sm.addEvent(event);
                    sm.waitForEvents(SimpleShooterState.FIRE);
                    break;

                case FIRE:
                    pidMotor.setTarget(oneShotDistance, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents((continuousModeOn)?SimpleShooterState.PULL_BACK:SimpleShooterState.DONE);
                    break;

                default:
                case DONE:
                    sm.stop();
                    setEnabled(false);
                    break;
            }
        }
    }

    //
    // Implements TrcPidController.PidInput
    //
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == this.pidCtrl)
        {
            input = shooterMotor.getPosition();
        }

        return input;
    }   //getInput
}
