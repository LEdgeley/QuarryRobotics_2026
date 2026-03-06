package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "First Path", group = "Autonomous")
@Configurable // Panels
public class FirstPath extends OpMode {
    private DcMotor back_intake;
    private DcMotor front_intake;

    private Limelight3A limelight;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Timer pathTimer;

    public static double limelightYawOffsetDeg = 90.0;
    public static double fieldHalfIn = 72.0;
    public static boolean flipY = false; // if true: Py = -LLy + 72, else Py = +LLy + 72

    public static double metersToInches = 39.3701;




    @Override
    public void init() {
        back_intake = hardwareMap.get(DcMotor.class, "back_intake");
        front_intake = hardwareMap.get(DcMotor.class, "front_intake");

        // Put initialization blocks here.
        back_intake.setDirection(DcMotor.Direction.REVERSE);
        front_intake.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second
        limelight.start();

        pathTimer = new Timer();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(52, 7, Math.toRadians(90)));

        limelight.updateRobotOrientation(0);

        LLResult result = limelight.getLatestResult();

        /*if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                Pose2D limelightPose = new Pose2D(DistanceUnit.METER,  botpose.getPosition().x,
                                                botpose.getPosition().y,
                                                AngleUnit.RADIANS,
                                                botpose.getOrientation().getYaw());
                Pose ftcStandard = PoseConverter.pose2DToPose(limelightPose, InvertedFTCCoordinates.INSTANCE);
                follower.setStartingPose(ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE));
            }
        }*/

        follower.update();
        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }



    @Override
    public void loop() {
        front_intake.setPower(0.9);
        back_intake.setPower(0.9);
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        double robotYaw = follower.getHeading();
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                panelsTelemetry.debug("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
        else {
            panelsTelemetry.debug("Status", "No April Tag found");
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public static PathChain Path1;
        public static PathChain Path2;
        public static PathChain Path3;
        public static PathChain Path4;
        public static PathChain Path5;
        public static PathChain Path6;
        public static PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(52.000, 7.000),
                                    new Pose(72.000, 71.776)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 71.776),
                                    new Pose(59.382, 85.045),
                                    new Pose(15.742, 84.611)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.742, 84.611),
                                    new Pose(33.511, 84.730),
                                    new Pose(54.380, 90.514),
                                    new Pose(72.000, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.000, 72.000),
                                    new Pose(56.255, 87.396),
                                    new Pose(59.727, 59.124),
                                    new Pose(14.963, 59.355)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(14.963, 59.355),
                                    new Pose(52.175, 60.181),
                                    new Pose(56.175, 87.403),
                                    new Pose(72.067, 71.868)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(72.067, 71.868),
                                    new Pose(51.502, 91.969),
                                    new Pose(46.535, 60.876),
                                    new Pose(72.025, 35.629),
                                    new Pose(13.880, 35.036)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(13.880, 35.036),
                                    new Pose(33.062, 35.102),
                                    new Pose(32.232, 56.893),
                                    new Pose(32.210, 72.099)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    /*these are the first path paths
    public static class Paths {

        public static PathChain Path1;
        public static PathChain Path2;


        public Paths(Follower follower) {
                    Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(39, 33),
                                    new Pose(72.583, 110.539),
                                    new Pose(105, 33)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .setTimeoutConstraint(200)
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(105, 33), new Pose(39, 33))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .setTimeoutConstraint(200)
                    .build();
        }
    }
*/
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(Paths.Path1);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Paths.Path2);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Paths.Path3);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Paths.Path4);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Paths.Path5);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Paths.Path6);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Paths.Path7);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain,grabPickup1,true we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
