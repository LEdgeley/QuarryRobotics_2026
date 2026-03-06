package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class QuarryRoboticsTeleOp extends OpMode {

    private Flywheel flywheel = new Flywheel(hardwareMap);
    private Intakes intakes = new Intakes(hardwareMap);
    private Finger front_finger = new Finger(hardwareMap, "front_finger", 0, 0);
    private Finger middle_finger =  new Finger(hardwareMap, "front_finger", 0, 0);;
    private Finger rear_finger = new Finger(hardwareMap, "front_finger", 0, 0);;

    private Finger[] fingers = {front_finger, middle_finger, rear_finger};

    private int current_finger = 0;

    @Override

    public void init() {

        //this is going to need some mods!
        follower.setStartingPose(new Pose(72,72));
        flywheel.set_speed(0);
        intakes.stop_intake();
        for (Finger finger: fingers) {
            finger.reset();
        }


    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {

        if (gamepad2.circleWasPressed())
        {
            fingers[current_finger].launch();
            current_finger = current_finger + 1 % 3;
        }

        if (gamepad2.leftBumperWasPressed())
        {
            if (intakes.running)
            {
                intakes.stop_intake();
            }
            else
            {
                intakes.start_intake();
            }

        }

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        for (Finger finger: fingers) {
            finger.update();
        }

        draw();
    }
}