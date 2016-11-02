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

public class Shooter implements TrcTaskMgr.Task
{
    private String instanceName;
    private FtcDcMotor shooterMotor;

    private enum ShooterState
    {
        ARM_AND_FIRE,
        DELAY,
        STOPPED
    }   //enum State

    private TrcTimer timer;
    private TrcEvent event;
    private TrcStateMachine sm;

    //the mechanism is to let motor rotate 360 degrees for each shot
    //the exact stop(next start) position for the motor doesn't matter
    //the below logic relies on getPosition can accurately tell the ticks for one rotation
    //otherwise we need to either calibrate or implement some sensor to detect the full rotation
    //or the other idea is to use motor speed to detect the teeth disengaging.
    //note: encoder cable must be plugged in with red wire up
    private static double PART_ACCEL_TRAVEL_DISTANCE_PER_ROTATION = 1667.0; //distance for one rotation
    private static double PART_ACCEL_DELAY_TIME_IN_SEC = 5.0;   //delay between shots if shooting continuously
    private static double PART_ACCEL_MOTOR_POWER = 1.0; //motor power
    private double nextStopPosition;
    private boolean shootContinuously;

    public Shooter(String instanceName){
        this.instanceName = instanceName;
        shooterMotor = new FtcDcMotor("shooterMotor");
        timer = new TrcTimer(instanceName);
        event = new TrcEvent(instanceName);
        sm = new TrcStateMachine(instanceName);
        shootContinuously = false;
        nextStopPosition = 0.0;
        shooterMotor.setInverted(true);
        shooterMotor.setSpeedTaskEnabled(true);
    }

    public double getSpeed() {
        return shooterMotor.getSpeed();
    }

    public void reset(){
        shooterMotor.resetPosition();
        shooterMotor.setPower(0.0);
        nextStopPosition = PART_ACCEL_TRAVEL_DISTANCE_PER_ROTATION;
    }

    public void openFire(boolean shootContinuously){
        shootContinuously = shootContinuously;
        shooterMotor.setPower(PART_ACCEL_MOTOR_POWER);
        //_setEnabled(true);
    }

    public void stopFire() {
        shootContinuously = false;
        FtcOpMode.getOpModeTracer().traceInfo("Robot","Shooter motor position (%.0f %.0f).",
                shooterMotor.getPosition(), nextStopPosition);
        shooterMotor.setPower(0.0);
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
        if (sm.isReady())
        {
            ShooterState state = (ShooterState)sm.getState();
            switch (state)
            {
                case ARM_AND_FIRE:
                    shooterMotor.setPower(PART_ACCEL_MOTOR_POWER);
                    if (shooterMotor.getPosition() > nextStopPosition) {
                        if (shootContinuously) {
                            sm.setState(ShooterState.DELAY);
                        }
                        else
                        {
                            sm.setState(ShooterState.STOPPED);
                        }
                        nextStopPosition += PART_ACCEL_TRAVEL_DISTANCE_PER_ROTATION;
                    }
                    FtcOpMode.getOpModeTracer().traceInfo("Robot","Shooter motor position (%.0f %.0f).",
                            shooterMotor.getPosition(), nextStopPosition);
                    break;

                case DELAY:
                    shooterMotor.setPower(0.0);
                    timer.set(PART_ACCEL_DELAY_TIME_IN_SEC, event);
                    sm.addEvent(event);
                    if (shootContinuously) {
                        sm.waitForEvents(ShooterState.ARM_AND_FIRE);
                    }
                    else {
                        sm.setState(ShooterState.STOPPED);
                    }
                    break;

                case STOPPED:
                    shooterMotor.setPower(0.0);
                    sm.stop();
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
                    instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
            sm.start(ShooterState.ARM_AND_FIRE);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(
                    this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }
}
