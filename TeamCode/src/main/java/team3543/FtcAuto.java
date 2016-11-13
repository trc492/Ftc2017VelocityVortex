package team3543;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;
import trclib.TrcRobot;

@Autonomous(name="Autonomous", group="3543Auto")
public class FtcAuto extends FtcOpMode implements FtcMenu.MenuButtons
{
    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum StartPosition
    {
        NEAR,
        FAR
    }   //enum StartPosition

    public enum Strategy
    {
        DO_NOTHING,
        BEACON,
        DEFENSE
    }   //enum Strategy

    public enum BeaconButtons
    {
        NONE,
        NEAR_BEACON,
        FAR_BEACON,
        BOTH
    }   //enum BeaconButtons

    public enum BeaconOption
    {
        DO_NOTHING,
        PARK_CORNER,
        PARK_CENTER
    }   //enum BeaconOption

    private HalDashboard dashboard;
    private Robot robot;

    private TrcRobot.AutoStrategy autoStrategy = null;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private StartPosition startPos = StartPosition.NEAR;
    private double delay = 0.0;
    private Strategy strategy = Strategy.DO_NOTHING;
    private int shootParticles = 2;
    private BeaconButtons beaconButtons = BeaconButtons.BOTH;
    private double driveDistance = 0.0;
    private BeaconOption beaconOption = BeaconOption.DO_NOTHING;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.RunMode.AUTO_MODE);
        dashboard = robot.dashboard;
        //
        // Choice menus.
        //
        doMenus();
        //
        // Strategies.
        //
        switch (strategy)
        {
            case BEACON:
                autoStrategy = new AutoBeacon(
                        robot, alliance, startPos, delay, shootParticles, beaconButtons, beaconOption);
                break;

            case DEFENSE:
                autoStrategy = new AutoDefense(robot, delay, driveDistance);
                break;

            case DO_NOTHING:
            default:
                autoStrategy = null;
                break;
        }

        getGlobalTracer().traceInfo(
                getOpModeName(),
                "Strategy: %s(alliance=%s, startPos=%s, delay=%.0f, shootParticles=%d, beaconButton=%s, beaconOption=%s",
                strategy.toString(), alliance.toString(), startPos.toString(), delay, shootParticles,
                beaconButtons.toString(), beaconOption.toString());
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode()
    {
        dashboard.clearDisplay();
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoStrategy != null)
        {
            autoStrategy.autoPeriodic(elapsedTime);
        }
    }   //runContinuous

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton

    private void doMenus()
    {
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", null, this);
        FtcChoiceMenu startPosMenu = new FtcChoiceMenu("Start position:", allianceMenu, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay time:", startPosMenu, this,
                                                  0.0, 15.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu strategyMenu = new FtcChoiceMenu("Strategies:", delayMenu, this);
        FtcValueMenu distanceMenu = new FtcValueMenu("Distance:", strategyMenu, this,
                1.0, 10.0, 1.0, 1.0, " %.0f ft");
        FtcValueMenu shootParticlesMenu = new FtcValueMenu("Shoot particles:", strategyMenu, this,
                                                           0.0, 2.0, 1.0, 2.0, "%.0f");
        FtcChoiceMenu beaconButtonsMenu =
                new FtcChoiceMenu("Push beacon button:", shootParticlesMenu, this);
        FtcChoiceMenu beaconOptionMenu =
                new FtcChoiceMenu("Beacon options", beaconButtonsMenu, this);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, startPosMenu);

        startPosMenu.addChoice("Near", StartPosition.NEAR, delayMenu);
        startPosMenu.addChoice("Far", StartPosition.FAR, delayMenu);

        delayMenu.setChildMenu(strategyMenu);

        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING);
        strategyMenu.addChoice("Beacon", Strategy.BEACON, shootParticlesMenu);
        strategyMenu.addChoice("Defense", Strategy.DEFENSE, distanceMenu);

        shootParticlesMenu.setChildMenu(beaconButtonsMenu);

        beaconButtonsMenu.addChoice("None", BeaconButtons.NONE, beaconOptionMenu);
        beaconButtonsMenu.addChoice("Near", BeaconButtons.NEAR_BEACON, beaconOptionMenu);
        beaconButtonsMenu.addChoice("Far", BeaconButtons.FAR_BEACON, beaconOptionMenu);
        beaconButtonsMenu.addChoice("Both", BeaconButtons.BOTH, beaconOptionMenu);

        beaconOptionMenu.addChoice("Do nothing", BeaconOption.DO_NOTHING);
        beaconOptionMenu.addChoice("Park center", BeaconOption.PARK_CENTER);
        beaconOptionMenu.addChoice("Park corner", BeaconOption.PARK_CORNER);

        FtcMenu.walkMenuTree(allianceMenu);

        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();
        startPos = (StartPosition)startPosMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();
        strategy = (Strategy)strategyMenu.getCurrentChoiceObject();
        driveDistance = distanceMenu.getCurrentValue();
        shootParticles = (int)shootParticlesMenu.getCurrentValue();
        beaconButtons = (BeaconButtons)beaconButtonsMenu.getCurrentChoiceObject();
        beaconOption = (BeaconOption)beaconOptionMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "Auto Strategy: %s (%s)",
                                strategyMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(1, "Start position: %s", startPos.toString());
        dashboard.displayPrintf(2, "Delay = %.0f sec", delay);
        dashboard.displayPrintf(3, "Defense: distance=%.0f ft", driveDistance);
        dashboard.displayPrintf(4, "Beacon: ShootParticles=%d,BeaconButtons=%s,Option=%s",
                                shootParticles, beaconButtons.toString(), beaconOption.toString());
    }   //doMenus

}   //class FtcAuto
