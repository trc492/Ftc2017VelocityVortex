package team3543;

public class RobotInfo
{
    //
    // DriveBase subsystem.
    //
    public static final double DRIVE_KP                         = 0.03;
    public static final double DRIVE_KI                         = 0.0;
    public static final double DRIVE_KD                         = 0.0;
    public static final double DRIVE_KF                         = 0.0;
    public static final double DRIVE_TOLERANCE                  = 2.0;
    public static final double DRIVE_SETTLING                   = 0.2;
    public static final double DRIVE_INCHES_PER_COUNT           = (104.0/7416.5);

    public static final double GYRO_KP                          = 0.05;
    public static final double GYRO_KI                          = 0.0;
    public static final double GYRO_KD                          = 0.0;
    public static final double GYRO_KF                          = 0.0;
    public static final double GYRO_TOLERANCE                   = 2.0;
    public static final double GYRO_SETTLING                    = 0.2;

    //
    // Line following PID control.
    //
    public static final double SONAR_KP                         = 0.08;
    public static final double SONAR_KI                         = 0.0;
    public static final double SONAR_KD                         = 0.0;
    public static final double SONAR_KF                         = 0.0;
    public static final double SONAR_TOLERANCE                  = 0.2;
    public static final double SONAR_SETTLING                   = 0.2;
    public static final double SONAR_INCHES_PER_CM              = (1.0/2.54);
    public static final double SONAR_BEACON_DISTANCE            = 2.0;

    public static final double COLOR_KP                         = 0.125;
    public static final double COLOR_KI                         = 0.0;
    public static final double COLOR_KD                         = 0.0;
    public static final double COLOR_KF                         = 0.0;
    public static final double COLOR_TOLERANCE                  = 2.0;
    public static final double COLOR_SETTLING                   = 0.2;
    public static final double COLOR_BLACK                      = 0.0;
    public static final double COLOR_BLUE                       = 3.0;
    public static final double COLOR_RED                        = 10.0;
    public static final double COLOR_WHITE                      = 16.0;
    public static final double COLOR_DARK_LEVEL                 = 0.0;
    public static final double COLOR_WHITE_LEVEL                = 10.0;
    public static final double COLOR_LINE_EDGE_LEVEL            = ((COLOR_DARK_LEVEL + COLOR_WHITE_LEVEL)/2.0);
    public static final double COLOR_LINE_EDGE_DEADBAND         = (COLOR_LINE_EDGE_LEVEL*0.25);

    public static final double BALL_PICKUP_MOTOR_POWER          = 1.0;

    public static final double BUTTON_PUSHER_RETRACT_POSITION = 0.0;
    public static final double BUTTON_PUSHER_EXTEND_POSITION = 1.0;

    public static final double CONVEYOR_GATE_CLOSE_SERVO_POSITION = 165.0/255.0;
    public static final double CONVEYOR_GATE_OPEN_SERVO_POSITION = 140.0/255.0;
    public static final double CONVEYOR_MOTOR_POWER = 1.0;
}   //class RobotInfo
