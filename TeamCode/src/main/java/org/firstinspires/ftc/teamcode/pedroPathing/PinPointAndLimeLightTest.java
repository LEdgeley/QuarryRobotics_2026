
package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

@Configurable
@TeleOp
public class PinPointAndLimeLightTest extends OpMode {
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;

    Limelight3A limelight;
    private Follower follower;

    // start pose
    public static double startX = 39;
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

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        follower.update();
        follower.startTeleOpDrive();

        lastRelocalized.reset();
    }

    @Override
    public void init_loop()
    {
        follower.update();
        Pose currentPose = follower.getPose();
        telemetry.addData("Pedro Pose",
                String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();
        lastRelocalized.reset();
    }

    @Override
    public void start() {
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
        telemetry.addData("Pedro Pose",
                String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();



        // Feed Limelight yaw (deg)
        double pedroYawDeg = Math.toDegrees(follower.getPose().getHeading());

        telemetry.addData("pedroYaw (deg)", pedroYawDeg);

        limelight.updateRobotOrientation(pedroYawDeg + limelightYawOffsetDeg);

        // Read Limelight result
        LLResult result = limelight.getLatestResult();



        if (result != null && result.isValid()) {
            Pose3D botpose_mt1 = result.getBotpose();

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            telemetry.addData("Botpose_MT1 exists", botpose_mt1 != null);
            telemetry.addData("Botpose_MT2 exists", botpose_mt2 != null);

/*            if (botpose_mt1 != null) {
                // Raw MT2 meters
                double a = botpose_mt1.getPosition().x;
                double b = botpose_mt1.getPosition().y;
                double h = botpose_mt1.getOrientation().getYaw(AngleUnit.DEGREES);

                telemetry.addData("limeLight MT1 yaw (deg)", h);
                telemetry.addData("limelight MT1 x (metres)", a);
                telemetry.addData("limelight MT1 y (metres)", b);
            }*/


            if (botpose_mt1 != null) {
                // Raw MT1 meters
                double a = botpose_mt1.getPosition().x;
                double b = botpose_mt1.getPosition().y;

                telemetry.addData("limeLight yaw (deg)", botpose_mt1.getOrientation().getYaw());
                telemetry.addData("limelight x (metres)", a);
                telemetry.addData("limelight y (metres)", b);
                // Convert meters -> inches
                double llXIn = a * metersToInches;
                double llYIn = b * metersToInches;

                telemetry.addData("limelight x (inches)", llXIn);
                telemetry.addData("limelight y (inches)", llYIn);

                // Convert -> Pedro inches (your tested mapping)
                double visionX = (flipX ? (-llXIn) : llXIn) + fieldHalfIn;
                double visionY = (flipY ? (-llYIn) : llYIn) + fieldHalfIn;

                if (flipXY) {
                    double temp = visionX;
                    visionX = visionY;
                    visionY = temp;
                }

                telemetry.addData("Vision Pedro (in)", String.format("(%.1f, %.1f)", visionX, visionY));

                // if you can relocalize
                boolean okTime = (waitSeconds <= 0.0) || (lastRelocalized.seconds() >= waitSeconds);
                boolean canCorrect = okTime;
                String rejectReason = null;

                if(!visionEnabled)
                {
                    rejectReason = "Vision has been Disabled";
                    canCorrect = false;
                }

                if (!okTime) {
                    rejectReason = String.format("waiting %.2f/%.2f s", lastRelocalized.seconds(), waitSeconds);
                }

                if (canCorrect) {
                    Pose cur = follower.getPose();
                    double dx = visionX - cur.getX();
                    double dy = visionY - cur.getY();
                    double jump = Math.hypot(dx, dy);

                    telemetry.addData("Vision delta (in)",
                            String.format("(dx=%.1f, dy=%.1f) jump=%.1f", dx, dy, jump));

                    // if jump not too big
                    if (jump <= maxJumpIn) {
                        // Apply correction to the current pose (blended)
                        double newX = cur.getX() + kVision * dx;
                        double newY = cur.getY() + kVision * dy;

                        follower.setPose(new Pose(newX, newY, cur.getHeading()));
                        lastRelocalized.reset();

                        telemetry.addData("Vision applied",
                                String.format("k=%.2f -> (%.2f, %.2f)", kVision, newX, newY));
                    } else {
                        canCorrect = false;
                        rejectReason = String.format("jump %.1f > %.1f", jump, maxJumpIn);
                    }
                }
                if (!canCorrect && rejectReason != null) {
                    telemetry.addData("Vision rejected", rejectReason);
                }
            } else {
                telemetry.addData("Botpose_MT2", "NULL - Check pipeline configuration");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
}
