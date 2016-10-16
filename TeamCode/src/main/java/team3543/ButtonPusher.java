package team3543;

import ftclib.FtcServo;
import trclib.TrcServo;

/**
 * Created by jzheng on 10/16/2016.
 */

public class ButtonPusher {
    private String _instanceName;
    private FtcServo pusherServo;
    ButtonPusher(String instanceName) {
        _instanceName = instanceName;
        pusherServo = new FtcServo(_instanceName+"Servo");
    }
    public void reset(){
        pusherServo.setPosition(TrcServo.CONTINUOUS_SERVO_REVERSE_MAX);
    }
    public void extend(){
        pusherServo.setPosition(TrcServo.CONTINUOUS_SERVO_FORWARD_MAX);
    }
    public void retract(){
        reset();
    }
}
