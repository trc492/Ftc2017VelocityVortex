package team3543;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;

/**
 * Created by jzheng on 10/23/2016.
 */

public class Conveyor {
    private FtcDcMotor _conveyorMotor;
    private FtcServo _gateServo;
    private String _instanceName;

    Conveyor(String instanceName) {
        _instanceName = instanceName;
        _conveyorMotor = new FtcDcMotor("conveyorMotor");
        _gateServo = new FtcServo("gateServo");
    }

    public void reset() {
        _conveyorMotor.setPower(0.0);
        _conveyorMotor.resetPosition();
        _gateServo.setPosition(RobotInfo.CONVEYOR_GATE_CLOSE_SERVO_POSITION);
    }

    public void openGate() {
        _gateServo.setPosition(RobotInfo.CONVEYOR_GATE_OPEN_SERVO_POSITION);
    }

    public void closeGate() {
        _gateServo.setPosition(RobotInfo.CONVEYOR_GATE_CLOSE_SERVO_POSITION);
    }

    public void startConveyor() {
        _conveyorMotor.setPower(RobotInfo.CONVEYOR_MOTOR_POWER);
    }

    public void stopConveyor() {
        _conveyorMotor.setPower(0.0);
    }
}