package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Finger {

    private final ElapsedTime fingerTimer = new ElapsedTime();

    public enum FingerState{
        LOW,
        START_RAISE,
        RAISING,
        HIGH,
        START_LOWER,
        LOWERING

    }
    private FingerState fsmState = FingerState.LOW; // Current fsm state (state machine)

    private final Servo servo;
    public static double finger_down = 0;
    public static double finger_up = 0;

    public Finger(HardwareMap hardwareMap, String deviceName, double low_position, double high_position) {
        servo = hardwareMap.get(Servo.class, deviceName);
        finger_down = low_position;
        finger_up = high_position;
    }

    public void launch() {
        if (fsmState == FingerState.LOW)
        {
            fsmState = FingerState.START_RAISE;
            update();
        }
    }

    public void reset()
    {
        servo.setPosition(finger_down);
        fsmState = FingerState.LOW;
    }
    public void update()
    {
        switch (fsmState) {
            case LOW:
                break;
            case START_RAISE:
                servo.setPosition(finger_up);
                fingerTimer.reset();
                fsmState = FingerState.RAISING;
            case RAISING:
                if (fingerTimer.milliseconds() > 100) {
                    fsmState = FingerState.HIGH;
                }
                break;
            case HIGH:
                fsmState = FingerState.START_LOWER;
                break;
            case START_LOWER:
                servo.setPosition(finger_down);
                fingerTimer.reset();
                fsmState = FingerState.LOWERING;
                break;
            case LOWERING:
                if (fingerTimer.milliseconds() > 100) {
                    fsmState = FingerState.LOW;
                }
                break;
        }
    }
}