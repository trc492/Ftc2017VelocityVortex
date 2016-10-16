package team3543;

/**
 * Created by jzheng on 10/16/2016.
 */
import ftclib.FtcDcMotor;

public class PartAccel {
    private String _instanceName;
    public FtcDcMotor _partAccelMotor;
    PartAccel(String instanceName){
        _instanceName = instanceName;
        _partAccelMotor = new FtcDcMotor("shootMotor");
    }
    public void reset(){
        _partAccelMotor.resetPosition();
        _partAccelMotor.setPower(0.0);
    }
    public void shoot(){
        _partAccelMotor.setPower(0.5);
    }
}
