package team3543;

import ftclib.FtcDcMotor;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

public class Shooter implements TrcTaskMgr.Task, TrcPidController.PidInput
{
    private enum ShooterState
    {
        ARM_AND_FIRE,
        REENGAGE,
        PULL_BACK,
        DONE
    }   //enum State

    private String instanceName;
    private FtcDcMotor shooterMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private TrcStateMachine sm;
    private TrcEvent event;
    private boolean continuousModeOn = false;

    public Shooter(String instanceName)
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
            sm.stop();
        }
        setEnabled(false);
        continuousModeOn = false;
        shooterMotor.setPower(0.0);
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
            sm.start(ShooterState.ARM_AND_FIRE);
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
            ShooterState state = (ShooterState)sm.getState();
            switch (state)
            {
                case ARM_AND_FIRE:
                    shooterMotor.setPower(RobotInfo.SHOOTER_HIGH_POWER);
                    if (shooterMotor.getSpeed() > RobotInfo.SHOOTER_SPEED_HIGH_THRESHOLD)
                    {
                        shooterMotor.setPower(0.0);
                        sm.setState(ShooterState.REENGAGE);
                    }
                    break;

                case REENGAGE:
                    shooterMotor.setPower(RobotInfo.SHOOTER_LOW_POWER);
                    if (shooterMotor.getSpeed() < RobotInfo.SHOOTER_SPEED_LOW_THRESHOLD)
                    {
                        sm.setState(ShooterState.PULL_BACK);
                    }
                    break;

                case PULL_BACK:
                    pidMotor.setTarget(RobotInfo.SHOOTER_PULLBACK_TARGET, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(continuousModeOn? ShooterState.ARM_AND_FIRE: ShooterState.DONE);
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

}   //class Shooter
