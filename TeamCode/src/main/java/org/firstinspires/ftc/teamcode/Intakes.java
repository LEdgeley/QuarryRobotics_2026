package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intakes {

    private DcMotor front_intake;
    private DcMotor back_intake;
    public static double intake_speed = 0.9;
    public boolean running = false;

    public Intakes(HardwareMap hardwareMap) {
        front_intake = hardwareMap.get(DcMotor.class, "front_intake");
        back_intake = hardwareMap.get(DcMotor.class, "back_intake");

        front_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        back_intake.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void start_intake() {
        front_intake.setPower(0.9);
        back_intake.setPower(0.9);
        running = true;
    }

    public void stop_intake() {
        front_intake.setPower(0);
        back_intake.setPower(0);
        running = false;
    }
}