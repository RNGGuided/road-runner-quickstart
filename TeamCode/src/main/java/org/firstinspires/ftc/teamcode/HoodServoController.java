
package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class HoodServoController {
    private final CRServo hoodServo;
    private final AbsoluteAnalogEncoder encoder;
    private final PIDFController pid;

    private double minDeg, maxDeg;
    private double targetDeg;

    public HoodServoController(CRServo hoodServo, AbsoluteAnalogEncoder encoder,
                               PIDFCoefficients coeffs,
                               double minDeg, double maxDeg) {
        this.hoodServo = hoodServo;
        this.encoder = encoder;
        this.pid = new PIDFController(coeffs);
        this.minDeg = minDeg;
        this.maxDeg = maxDeg;
        this.targetDeg = minDeg;
    }

    public void setTargetDeg(double deg) {
        targetDeg = clamp(deg, minDeg, maxDeg);
    }

    public void update() {
        double currentDeg = encoder.getCurrentPosition(); // make sure this is 0-360 reading
        // IMPORTANT: do NOT wrap error
        double errorDeg = targetDeg - currentDeg;

        // If your encoder jumps near 0/360, force it into hood range:
        if (currentDeg < minDeg) currentDeg += 360.0;

        errorDeg = targetDeg - currentDeg;

        double power = pid.calculate(0, errorDeg);
        power = Range.clip(power, -1.0, 1.0);
        hoodServo.setPower(power);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
