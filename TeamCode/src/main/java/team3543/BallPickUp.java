package team3543;

import ftclib.FtcDcMotor;

/**
 * Created by jzheng on 10/16/2016.
 */

public class BallPickUp {
    private FtcDcMotor pickUpMotor;
    private String _InstanceName;
    BallPickUp(String instanceName){
        _InstanceName = instanceName;
        pickUpMotor = new FtcDcMotor("pickUpMotor");
        pickUpMotor.setInverted(true);
    }
    public void reset(){
        pickUpMotor.resetPosition();
        pickUpMotor.setPower(0.0);
    }
    public void startPickUp (){
        pickUpMotor.setPower(0.5);
    }
    public void stopPickUp () {
        reset();
    }

}
