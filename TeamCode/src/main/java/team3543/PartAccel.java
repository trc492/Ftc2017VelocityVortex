package team3543;

/**
 * Created by jzheng on 10/16/2016.
 */
import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
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

    //the mechanism is to let motor rotate 360 degrees for each shot
    //the exact stop(next start) position for the motor doesn't matter
    //the below logic relies on getPosition can accurately tell the ticks for one rotation
    //otherwise we need to either calibrate or implement some sensor to detect the full rotation
    //or the other idea is to use motor speed to detect the teeth disengaging.
    //note: encoder cable must be plugged in with red wire up
    private static double PART_ACCEL_TRAVEL_DISTANCE_PER_ROTATION = 1667.0; //distance for one rotation
    private static double PART_ACCEL_DELAY_TIME_IN_SEC = 5.0;   //delay between shots if shooting continuously
    private static double PART_ACCEL_MOTOR_POWER = 1.0; //motor power
    private double _nextStopPosition;
    private boolean _shootContinuously;

    PartAccel(String instanceName){
        _instanceName = instanceName;
        _partAccelMotor = new FtcDcMotor("shooterMotor");
        _timer = new TrcTimer(_instanceName);
        _event = new TrcEvent(_instanceName);
        _sm = new TrcStateMachine(_instanceName);
        _shootContinuously = false;
        _nextStopPosition = 0.0;
        _partAccelMotor.setInverted(true);
    }

    public void reset(){
        _partAccelMotor.resetPosition();
        _partAccelMotor.setPower(0.0);
        _nextStopPosition = PART_ACCEL_TRAVEL_DISTANCE_PER_ROTATION;
    }

    public void openFire(boolean shootContinuously){
        _shootContinuously = shootContinuously;
        _partAccelMotor.setPower(PART_ACCEL_MOTOR_POWER);
        //_setEnabled(true);
    }

    public void stopFire() {
        _shootContinuously = false;
        FtcOpMode.getOpModeTracer().traceInfo("Robot","Shooter motor position (%.0f %.0f).",
                _partAccelMotor.getPosition(),_nextStopPosition);
        _partAccelMotor.setPower(0.0);
        //it may run through FIRE_AND_ARM cycle one more to stop
        //need to check if there is a way to break from the DELAY state
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
                    _partAccelMotor.setPower(PART_ACCEL_MOTOR_POWER);
                    if (_partAccelMotor.getPosition() > _nextStopPosition) {
                        if (_shootContinuously) {
                            _sm.setState(ShooterState.DELAY);
                        }
                        else
                        {
                            _sm.setState(ShooterState.STOPPED);
                        }
                        _nextStopPosition += PART_ACCEL_TRAVEL_DISTANCE_PER_ROTATION;
                    }
                    FtcOpMode.getOpModeTracer().traceInfo("Robot","Shooter motor position (%.0f %.0f).",
                            _partAccelMotor.getPosition(),_nextStopPosition);
                    break;

                case DELAY:
                    _partAccelMotor.setPower(0.0);
                    _timer.set(PART_ACCEL_DELAY_TIME_IN_SEC, _event);
                    _sm.addEvent(_event);
                    if (_shootContinuously) {
                        _sm.waitForEvents(ShooterState.ARM_AND_FIRE);
                    }
                    else {
                        _sm.setState(ShooterState.STOPPED);
                    }
                    break;

                case STOPPED:
                    _partAccelMotor.setPower(0.0);
                    _sm.stop();
                    _setEnabled(false);
                    break;
            }
        }
    }

    private void _setEnabled(boolean enabled)
    {
        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(
                    _instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            _sm.start(ShooterState.ARM_AND_FIRE);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(
                    this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }
}
