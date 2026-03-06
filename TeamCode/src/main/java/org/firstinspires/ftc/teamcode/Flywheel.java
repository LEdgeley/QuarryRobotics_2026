package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class Flywheel {

    private DcMotorEx flywheel;
    public String state = "stopped";

    public Flywheel(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        // Feedforward gain to counteract constant forces like friction.
        double f = 14.098;
        // Proportional gain to correct error based on how far off the velocity is.
        double p = 200;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel.setVelocity(0);
        this.state = "stopped";
    }

    public void set_speed(double ticks_per_second)
    {
        flywheel.setVelocity(ticks_per_second);
        if (ticks_per_second > 0)
        {
            state = "running";
        }
        else {
            state = "stopped";
        }
    }

    public void get_speed()
    {
        flywheel.getVelocity();
    }

}