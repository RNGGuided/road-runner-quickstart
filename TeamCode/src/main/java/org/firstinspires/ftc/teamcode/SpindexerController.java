package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SpindexerController {

    // --------------------------------------------------
    // Modes
    // --------------------------------------------------
    public enum Mode {
        SHORTEST,
        FORCE_CW,
        FORCE_CCW
    }

    // --------------------------------------------------
    // Hardware
    // --------------------------------------------------
    private final CRServo servo;
    private final AnalogInput encoder;

    // --------------------------------------------------
    // Slot positions (0–1 normalized)
    // --------------------------------------------------
    private final double[] slotPositions;

    // --------------------------------------------------
    // PD + Feedforward (Dashboard Tunable)
    // --------------------------------------------------
    public static double kP = 0.004;
    public static double kD = 0.00035;

    public static double kStatic = 0.05;   // ⚡ SPEED BOOST

    public static double maxPower = 1.0;
    public static double minPower = 0.0;    
    public static double errorDeadband = 2.0; // degrees

    // --------------------------------------------------
    // Encoder settings
    // --------------------------------------------------
    private double offsetDeg = 0.0;
    private boolean invert = false;

    // --------------------------------------------------
    // State
    // --------------------------------------------------
    private Mode currentMode = Mode.SHORTEST;
    private double targetDeg = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;

    // --------------------------------------------------
    // BURST MODE (autoshoot kick)
    // --------------------------------------------------
    private boolean burstActive = false;
    private double burstPower = 0;
    private double burstDurationMs = 0;
    private final ElapsedTime burstTimer = new ElapsedTime();

    // --------------------------------------------------
    // Constructor
    // --------------------------------------------------
    public SpindexerController(CRServo servo,
                               AnalogInput encoder,
                               double[] slotPositions) {

        this.servo = servo;
        this.encoder = encoder;
        this.slotPositions = slotPositions;

        timer.reset();
    }

    // --------------------------------------------------
    // Configuration
    // --------------------------------------------------
    public void setOffsetDeg(double offsetDeg) {
        this.offsetDeg = offsetDeg;
    }

    public void setInvert(boolean invert) {
        this.invert = invert;
    }

    // --------------------------------------------------
    // Slot Commands
    // --------------------------------------------------
    public void rotateCWToSlot(int slotIndex) {
        targetDeg = getSlotDeg(slotIndex);
        currentMode = Mode.FORCE_CW;
        lastError = 0;
    }

    public void rotateCCWToSlot(int slotIndex) {
        targetDeg = getSlotDeg(slotIndex);
        currentMode = Mode.FORCE_CCW;
        lastError = 0;
    }

    public void rotateShortestToSlot(int slotIndex) {
        targetDeg = getSlotDeg(slotIndex);
        currentMode = Mode.SHORTEST;
        lastError = 0;
    }

    // --------------------------------------------------
    // BURST FUNCTION (FULL SEND)
    // --------------------------------------------------
    public void shootBurst(int direction, double durationMs) {

        burstActive = true;
        burstPower = Range.clip(direction, -1, 1);
        burstDurationMs = durationMs;
        burstTimer.reset();
    }

    // --------------------------------------------------
    // MAIN UPDATE LOOP
    // --------------------------------------------------
    public void update() {

        // ---------------- BURST OVERRIDE ----------------
        if (burstActive) {
            if (burstTimer.milliseconds() < burstDurationMs) {
                servo.setPower(burstPower);
                return;
            } else {
                burstActive = false;
                servo.setPower(0);
            }
        }

        // ---------------- PD LOOP ----------------
        double now = timer.seconds();
        double dt = now - lastTime;
        lastTime = now;

        if (dt <= 0.0001 || dt > 0.5) return;

        double currentDeg = getCurrentAngle();
        double error = computeErrorDeg(targetDeg, currentDeg);

        // Deadband hold
        if (Math.abs(error) < errorDeadband) {
            servo.setPower(0);
            lastError = error;
            return;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kD * derivative);

        // ⚡ Static feedforward for speed
        power += Math.copySign(kStatic, error);

        power = Range.clip(power, -maxPower, maxPower);

        if (Math.abs(power) < minPower) {
            power = Math.copySign(minPower, power);
        }

        servo.setPower(power);
    }

    // --------------------------------------------------
    // ANGLE FROM ANALOG ENCODER
    // --------------------------------------------------
    private double getCurrentAngle() {

        double deg = (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;

        deg -= offsetDeg;
        deg = normalizeDeg(deg);

        if (invert) {
            deg = normalizeDeg(360.0 - deg);
        }

        return deg;
    }

    // --------------------------------------------------
    // ERROR LOGIC IN DEGREES
    // --------------------------------------------------
    private double computeErrorDeg(double target, double current) {

        double error = target - current;

        switch (currentMode) {

            case SHORTEST:
                if (error > 180) error -= 360;
                if (error < -180) error += 360;
                break;

            case FORCE_CW:
                if (Math.abs(error) > 50) {
                    if (error < 0) error += 360;
                } else {
                    if (error > 180) error -= 360;
                    if (error < -180) error += 360;
                }
                break;

            case FORCE_CCW:
                if (Math.abs(error) > 50) {
                    if (error > 0) error -= 360;
                } else {
                    if (error > 180) error -= 360;
                    if (error < -180) error += 360;
                }
                break;
        }

        return error;
    }

    // --------------------------------------------------
    // SLOT ARRAY ACCESS
    // --------------------------------------------------
    private double getSlotDeg(int slotIndex) {

        int i = ((slotIndex % slotPositions.length)
                + slotPositions.length) % slotPositions.length;

        return normalizeDeg(slotPositions[i] * 360.0);
    }

    // --------------------------------------------------
    // UTIL
    // --------------------------------------------------
    private double normalizeDeg(double deg) {

        deg %= 360.0;
        if (deg < 0) deg += 360.0;

        return deg;
    }

    public void stop() {
        servo.setPower(0);
    }
}
