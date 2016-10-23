package team3543;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;

/**
 * Created by jzheng on 10/23/2016.
 */

public class Conveyer {
    private FtcDcMotor _conveyerMotor;
    private FtcServo _gateServo;
    private String _instanceName;

    private static double CONVEYER_GATE_CLOSE_SERVO_POSITION=0.0;
    private static double CONVEYER_GATE_OPEN_SERVO_POSITION=0.0;
    private static double CONVEYER_MOTOR_POWER=1.0;

    Conveyer(String instanceName) {
        _instanceName = instanceName;
        _conveyerMotor = new FtcDcMotor("conveyerMotor");
        _gateServo = new FtcServo("gateServo");
    }

    public void reset() {
        _conveyerMotor.setPower(0.0);
        _conveyerMotor.resetPosition();
        _gateServo.setPosition(CONVEYER_GATE_CLOSE_SERVO_POSITION);
    }

    public void openGate(){
        _gateServo.setPosition(CONVEYER_GATE_OPEN_SERVO_POSITION);
    }

    public void closeGate() {
        _gateServo.setPosition(CONVEYER_GATE_CLOSE_SERVO_POSITION);
    }

    public void startConveyer(){
        _conveyerMotor.setPower(CONVEYER_MOTOR_POWER);
    }

    public void stopConveyer(){
        _conveyerMotor.setPower(0.0);
    }
}
