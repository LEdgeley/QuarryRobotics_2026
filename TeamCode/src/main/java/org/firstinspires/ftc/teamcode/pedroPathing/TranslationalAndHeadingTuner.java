package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This is the Translational And Heading PIDF Tuner OpMode. It will keep the robot in place and at the same heading
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class TranslationalAndHeadingTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the translational PIDF(s)");
        telemetryM.debug("The robot will try to stay in place while you push it laterally.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        follower.activateHeading();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
        telemetryM.update(telemetry);
    }
}

