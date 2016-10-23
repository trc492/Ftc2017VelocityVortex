package team3543;

/**
 * Created by jzheng on 10/16/2016.
 */
import ftclib.FtcDcMotor;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

public class PartAccel implements TrcTaskMgr.Task
{
    private String _instanceName;
    private FtcDcMotor _partAccelMotor;

    private enum ShooterState
    {
        ARM_AND_FIRE,
        DELAY,
        STOPPED
    }   //enum State

    private TrcTimer _timer;
    private TrcEvent _event;
    private TrcStateMachine _sm;
    private static double PART_ACCEL_TRAVEL_DISTANCE = 10.0;
    private static double PART_ACCEL_DELAY_TIME_IN_SEC = 2.0;

    PartAccel(String instanceName){
        _instanceName = instanceName;
        _partAccelMotor = new FtcDcMotor("shooterMotor");
        _timer = new TrcTimer(_instanceName);
        _event = new TrcEvent(_instanceName);
        _sm = new TrcStateMachine(_instanceName);
    }

    public void reset(){
        _partAccelMotor.resetPosition();
        _partAccelMotor.setPower(0.0);
    }
    public void shoot(){
        _partAccelMotor.setPower(0.5);
    }

    @Override
    public void startTask(TrcRobot.RunMode runMode) {

    }

    @Override
    public void stopTask(TrcRobot.RunMode runMode) {

    }

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode) {

    }

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode) {

    }

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode) {

    }

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode) {
        if (_sm.isReady())
        {
            ShooterState state = (ShooterState)_sm.getState();
            switch (state)
            {
                case ARM_AND_FIRE:
                    _partAccelMotor.setPower(1.0);
                    if (_partAccelMotor.getPosition() > PART_ACCEL_TRAVEL_DISTANCE) {
                        _sm.setState(ShooterState.DELAY);
                    }
                    break;

                case DELAY:
                    _partAccelMotor.setPower(0.0);
                    _timer.set(PART_ACCEL_DELAY_TIME_IN_SEC, _event);
                    _sm.addEvent(_event);
                    _sm.waitForEvents(ShooterState.ARM_AND_FIRE);
                    break;

                case STOPPED:
                    _partAccelMotor.setPower(0.0);
                    _sm.stop();
                    break;
            }
        }
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(
                    _instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(
                    this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }
}
