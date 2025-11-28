package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;

import java.util.function.BooleanSupplier;

public class ShooterActions {

    private final ShooterSystem shooter;
    private final int[] spindexerBalls;

    public ShooterActions(ShooterSystem shooter, int[] spindexerBalls) {
        this.shooter = shooter;
        this.spindexerBalls = spindexerBalls;
    }

    // --------------------------------------------------------
    // BALL TRACKING (color sensors)
    // --------------------------------------------------------
    public void updateBallTracking() {
        ShooterSystem.DetectedColor col = shooter.getDetectedColor(null);
        if (col == ShooterSystem.DetectedColor.UNKNOWN) return;

        int index = shooter.currentTargetIndex;
        if (index == -1) index = 0;

        if (col == ShooterSystem.DetectedColor.GREEN)
            spindexerBalls[index] = 1;
        else if (col == ShooterSystem.DetectedColor.PURPLE)
            spindexerBalls[index] = 2;
    }

    // --------------------------------------------------------
    // SHOOTER MODE
    // --------------------------------------------------------
    public Action setShooterMode(ShooterSystem.ShooterControlMode mode) {
        return p -> {
            shooter.setShooterMode(mode);
            return false;
        };
    }

    // --------------------------------------------------------
    // SPIN SHOOTER TO RPM
    // --------------------------------------------------------
    public Action spinUpRpm(double rpm) {
        return p -> {
            shooter.setShooterTargetRpm(rpm);
            return false;
        };
    }

    public Action stopShooterRpm() {
        return p -> {
            shooter.stopShooterBangBang();
            return false;
        };
    }

    // --------------------------------------------------------
    // KICKER ACTIONS
    // --------------------------------------------------------
    public Action kickerDown() {
        return p -> {
            shooter.KickerDown();
            return false;
        };
    }

    public Action kickerUp() {
        return p -> {
            shooter.KickerUp();
            return false;
        };
    }

    // --------------------------------------------------------
    // INTAKE ACTIONS
    // --------------------------------------------------------
    public Action intakeForward(double pwr) {
        return p -> {
            shooter.intakeOn(pwr);
            return false;
        };
    }

    public Action intakeReverse(double pwr) {
        return p -> {
            shooter.intakeReverse(pwr);
            return false;
        };
    }

    public Action intakeOff() {
        return p -> {
            shooter.intakeOff();
            return false;
        };
    }

    // --------------------------------------------------------
    // INDEXER (CW)
    // --------------------------------------------------------
    public Action indexNextAngle(double timeoutSeconds) {
        return new SequentialAction(
                p -> { shooter.spinToNextAngle(); return false; },
                waitUntil(() -> !shooter.isSpinning(), timeoutSeconds, true)
        );
    }

    // --------------------------------------------------------
    // INDEXER (CCW)
    // --------------------------------------------------------
    public Action indexNextAngleReverse(double timeoutSeconds) {
        return new SequentialAction(
                p -> { shooter.spinToNextAngle2(); return false; },
                waitUntil(() -> !shooter.isSpinning2(), timeoutSeconds, true)
        );
    }

    // --------------------------------------------------------
    // FEEDER (full)
    // --------------------------------------------------------
    public Action feedFull(double seconds) {
        return new SequentialAction(
                p -> { shooter.controlFeeder(-1, 0); return false; },
                new SleepAction(seconds),
                p -> { shooter.controlFeeder(0, 0); return false; }
        );
    }

    // --------------------------------------------------------
    // KEEP UPDATING SHOOTER + INDEXER + COLOR SENSOR
    // --------------------------------------------------------
    public Action keepUpdatingFor(double seconds) {
        return new Action() {
            long startNs = -1;

            @Override
            public boolean run(TelemetryPacket p) {

                if (startNs == -1) startNs = System.nanoTime();

                shooter.update();                // CW indexer
                shooter.update2();               // CCW indexer
                shooter.updateServo();           // servo indexer
                shooter.updateShooterBangBang(); // shooter control
                updateBallTracking();            // color tracking

                double t = (System.nanoTime() - startNs) / 1e9;
                return t < seconds;
            }
        };
    }

    // --------------------------------------------------------
    // WAIT UNTIL CONDITION (while still updating shooter!)
    // --------------------------------------------------------
    public Action waitUntil(BooleanSupplier done, double timeoutSeconds, boolean doUpdates) {

        return new Action() {

            long startNs = -1;

            @Override
            public boolean run(TelemetryPacket p) {

                if (startNs == -1) startNs = System.nanoTime();

                if (doUpdates) {
                    shooter.update();
                    shooter.update2();
                    shooter.updateServo();
                    shooter.updateShooterBangBang();
                    updateBallTracking();
                }

                double t = (System.nanoTime() - startNs) / 1e9;

                return !(done.getAsBoolean() || t >= timeoutSeconds);
            }
        };
    }
}
