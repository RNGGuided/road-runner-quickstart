package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HoodServoController {

    private final Servo hoodServo;

    // Servo range limits (0–1)
    private final double minPos;
    private final double maxPos;

    private double targetPos;

    public HoodServoController(
            Servo hoodServo,
            double minPos,
            double maxPos
    ) {
        this.hoodServo = hoodServo;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.targetPos = minPos;
    }

    /** Set desired hood position (0–1) */
    public void setTargetPos(double pos) {
        targetPos = Range.clip(pos, minPos, maxPos);
    }

    /** Call every loop */
    public void update() {
        hoodServo.setPosition(targetPos);
    }

    public double getTargetPos() {
        return targetPos;
    }
}
