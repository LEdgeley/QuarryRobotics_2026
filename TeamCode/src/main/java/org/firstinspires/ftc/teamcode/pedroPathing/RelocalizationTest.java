
package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ExponentialMovingAverage;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
@TeleOp
public class RelocalizationTest extends OpMode {

    private Limelight3A limelight;

    private ExponentialMovingAverage xEMA;
    private ExponentialMovingAverage yEMA;
    private ExponentialMovingAverage hEMA;

    private Pose emaPose;

    public Follower follower; // Pedro Pathing follower instance

    // start pose
    public static double startX = 37.5;
    public static double startY = 33;
    public static double startHeadingDeg = 90;

    // how much time in between relocalizing
    public static double waitSeconds = 0.0;

    public static boolean visionEnabled = true;

    // kalman filter
    public static double kVision = 0.7;      // 0.15–0.50 typical

    // max distance to relocalize to
    public static double maxJumpIn = 24.0;

    // field conversion shi
    public static double limelightYawOffsetDeg = 90;
    public static double fieldHalfIn = 72.0;
    public static boolean flipY = false; // if true: Py = -LLy + 72, else Py = +LLy + 72
    public static boolean flipX = true; // if true: Px = -LLx + 72, else Px = +LLx + 72
    public static boolean flipXY = true; // if true: swap X and Y

    public static double metersToInches = 39.3701;

    private final ElapsedTime lastRelocalized = new ElapsedTime();

    public void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public void drawEMA(Pose emaPose) {
        try {
            Drawing.drawRobot(emaPose, new Style(
                    "", "#B53F51", 0.75
            ));
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public void drawMT2(Pose mt2Pose) {
        try {
            Drawing.drawRobot(mt2Pose, new Style(
                    "", "#51B53F", 0.75
            ));
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }



    @Override
    public void init() {
        xEMA = new ExponentialMovingAverage(0.01);
        yEMA = new ExponentialMovingAverage(0.01);
        hEMA = new ExponentialMovingAverage(0.01);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        follower.update();
        follower.startTeleOpDrive();
        limelight.start();
    }

    @Override
    public void init_loop()
    {
        follower.update();
        Pose currentPose = follower.getPose();
        Pose mt2CorrectedPose = null;

        telemetry.addData("PinPoint Pose",
                String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        limelight.updateRobotOrientation(Math.toDegrees(currentPose.getHeading()) + limelightYawOffsetDeg);

        LLResult result = limelight.getLatestResult();

        if(result != null){
            if (result.isValid())
            {
                Pose3D mt2Pose = result.getBotpose_MT2();

                if (mt2Pose != null) {
                    // Raw MT2 meters
                    double a = mt2Pose.getPosition().x;
                    double b = mt2Pose.getPosition().y;

                    // Convert meters -> inches
                    double llXIn = a * metersToInches;
                    double llYIn = b * metersToInches;

                    // Convert -> Pedro inches (your tested mapping)
                    double visionX = (flipX ? (-llXIn) : llXIn) + fieldHalfIn;
                    double visionY = (flipY ? (-llYIn) : llYIn) + fieldHalfIn;

                    if (flipXY) {
                        double temp = visionX;
                        visionX = visionY;
                        visionY = temp;
                    }

                    mt2CorrectedPose = new Pose(visionX, visionY, Math.toRadians(mt2Pose.getOrientation().getYaw(AngleUnit.DEGREES) - limelightYawOffsetDeg));
                    telemetry.addData("MT2 Pose",
                            String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                                    mt2CorrectedPose.getX(), mt2CorrectedPose.getY(), Math.toDegrees(mt2CorrectedPose.getHeading())));
                    drawMT2(mt2CorrectedPose);

                    emaPose = new Pose(xEMA.calculateEMA(mt2CorrectedPose.getX()),
                                    yEMA.calculateEMA(mt2CorrectedPose.getY()),
                                    hEMA.calculateEMA(mt2CorrectedPose.getHeading()));
                    drawEMA(emaPose);
                    telemetry.addData("EMA Pose",
                            String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                                    emaPose.getX(), emaPose.getY(), Math.toDegrees(emaPose.getHeading())));
                }


            }
        }


        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();
        lastRelocalized.reset();
    }

    @Override
    public void start() {
        follower.setPose(emaPose);
        limelight.start();
        follower.startTeleopDrive();
        //follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        lastRelocalized.reset();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // Driver control

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        Pose currentPose = follower.getPose();
        Pose mt2CorrectedPose = null;

        telemetry.addData("PinPoint Pose",
                String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        limelight.updateRobotOrientation(Math.toDegrees(currentPose.getHeading()) + limelightYawOffsetDeg);

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D mt2Pose = result.getBotpose_MT2();

                if (mt2Pose != null) {
                    // Raw MT2 meters
                    double a = mt2Pose.getPosition().x;
                    double b = mt2Pose.getPosition().y;

                    // Convert meters -> inches
                    double llXIn = a * metersToInches;
                    double llYIn = b * metersToInches;

                    // Convert -> Pedro inches (your tested mapping)
                    double visionX = (flipX ? (-llXIn) : llXIn) + fieldHalfIn;
                    double visionY = (flipY ? (-llYIn) : llYIn) + fieldHalfIn;

                    if (flipXY) {
                        double temp = visionX;
                        visionX = visionY;
                        visionY = temp;
                    }

                    mt2CorrectedPose = new Pose(visionX, visionY, Math.toRadians(mt2Pose.getOrientation().getYaw(AngleUnit.DEGREES) - limelightYawOffsetDeg));
                    telemetry.addData("MT2 Pose",
                            String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                                    mt2CorrectedPose.getX(), mt2CorrectedPose.getY(), Math.toDegrees(mt2CorrectedPose.getHeading())));
                    drawMT2(mt2CorrectedPose);

                    emaPose = new Pose(xEMA.calculateEMA(mt2CorrectedPose.getX()),
                            yEMA.calculateEMA(mt2CorrectedPose.getY()),
                            hEMA.calculateEMA(mt2CorrectedPose.getHeading()));
                    drawEMA(emaPose);
                    telemetry.addData("EMA Pose",
                            String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                                    emaPose.getX(), emaPose.getY(), Math.toDegrees(emaPose.getHeading())));
                }


            }
        }

        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();
        lastRelocalized.reset();
    }
}
