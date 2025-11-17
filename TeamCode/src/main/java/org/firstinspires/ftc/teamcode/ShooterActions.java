package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import java.util.function.BooleanSupplier;

/**
 * Drop-in helper that wraps your ShooterSystem in Road Runner v1.0 Actions so you can
 * compose them inside your autonomous sequences (SequentialAction/ParallelAction).
 *
 * Usage (inside your auton):
 *
 *   ShooterSystem shooter = new ShooterSystem(hardwareMap);
 *   ShooterActions SA = new ShooterActions(shooter);
 *
 *   Action auto = new SequentialAction(
 *       SA.captureOptimalAngles(),           // optional once at start
 *       SA.spinUp(0.90),
 *       new SleepAction(1.2),                // give flywheels time
 *       new ParallelAction(
 *           path,                            // your RR path Action
 *           SA.keepUpdatingFor(99.0)         // keeps update() / updateServo() running while driving
 *       ),
 *       new SequentialAction(
 *           SA.indexNextAngle(0.8),          // CR-spindexer to next slot, wait until done or timeout
 *           SA.feedFor(0.35, +1.0),          // feed one ring forward for 0.35s
 *           SA.indexNextAngle(0.8),
 *           SA.feedFor(0.35, +1.0),
 *           SA.indexNextAngle(0.8),
 *           SA.feedFor(0.35, +1.0)
 *       ),
 *       SA.stopAll()
 *   );
 *
 *   Actions.runBlocking(auto);
 */
public class ShooterActions {
    private final ShooterSystem shooter;

    public ShooterActions(ShooterSystem shooter) {
        this.shooter = shooter;
    }

    /** Instant: call getOptimalAngles() once (it internally guards to only set once). */
    public Action captureOptimalAngles() {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                shooter.getOptimalAngles();
                return false; // instant
            }
        };
    }

    /** Instant: start flywheels at power. */
    public Action spinUp(double power) {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                shooter.shoot(power);
                return false; // instant
            }
        };
    }

    /** Instant: stop everything shooter-related (flywheels + feeder off, kicker down). */
    public Action stopAll() {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                shooter.stopShooter();
                shooter.controlFeeder(0, 0);
                shooter.KickerDown();
                shooter.intakeOff();
                return false; // instant
            }
        };
    }

    /** Instant: kicker up. */
    public Action kickerUp() {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                shooter.KickerUp();
                return false;
            }
        };
    }

    /** Instant: kicker down. */
    public Action kickerDown() {
        return new Action() {
            @Override public boolean run(TelemetryPacket p) {
                shooter.KickerDown();
                return false;
            }
        };
    }

    /** Feed using your trigger-style API: positive power feeds forward, negative reverse. */
    public Action feedFor(double seconds, double forwardPower01) {
        double fwd = Math.max(0.0, Math.min(1.0, forwardPower01));
        // controlFeeder(rightTrigger, leftTrigger) -> forward = right - left
        return new SequentialAction(
                kickerUp(),
                new Action() { // start feeder
                    @Override public boolean run(TelemetryPacket p) {
                        shooter.controlFeeder(fwd, 0.0);
                        return false;
                    }
                },
                new SleepAction(seconds),
                new Action() { // stop feeder and drop kicker
                    @Override public boolean run(TelemetryPacket p) {
                        shooter.controlFeeder(0.0, 0.0);
                        shooter.KickerDown();
                        return false;
                    }
                }
        );
    }
    public Action feedFull(double seconds) {
        return new SequentialAction(
                new Action() {
                    @Override public boolean run(TelemetryPacket p) {
                        shooter.controlFeeder(-1.0, 0.0); // full power forward
                        return false;
                    }
                },
                new SleepAction(seconds),
                new Action() {
                    @Override public boolean run(TelemetryPacket p) {
                        shooter.controlFeeder(0.0, 0.0); // stop
                        return false;
                    }
                }
        );
    }

    /** Intake helpers */
    public Action intakeForward(double power01) {
        double p = Math.max(0.0, Math.min(1.0, power01));
        return new Action() { @Override public boolean run(TelemetryPacket pkt) { shooter.intakeOn(p); return false; } };
    }
    public Action intakeReverse(double power01) {
        double p = Math.max(0.0, Math.min(1.0, power01));
        return new Action() { @Override public boolean run(TelemetryPacket pkt) { shooter.intakeReverse(p); return false; } };
    }
    public Action intakeOff() {
        return new Action() { @Override public boolean run(TelemetryPacket pkt) { shooter.intakeOff(); return false; } };
    }

    /**
     * Start CR spindexer toward next pre-defined angle (CW) and wait until it finishes or timeout.
     * Calls shooter.update() repeatedly in the wait loop.
     */
    public Action indexNextAngle(double timeoutSeconds) {
        return new SequentialAction(
                new Action() {
                    @Override public boolean run(TelemetryPacket p) {
                        shooter.spinToNextAngle(0 /* currentAngle unused */);
                        return false;
                    }
                },
                waitUntil(() -> !shooter.isSpinning(), timeoutSeconds, true)
        );
    }

    /**
     * Same idea, but for your servo-based indexing PID (spinToNextAngleServo / isSpinningServo).
     */
    public Action indexNextAngleServo(double timeoutSeconds) {
        return new SequentialAction(
                new Action() {
                    @Override public boolean run(TelemetryPacket p) {
                        shooter.spinToNextAngleServo();
                        return false;
                    }
                },
                waitUntil(() -> !shooter.isSpinningServo(), timeoutSeconds, true)
        );
    }

    /** Keep running update loops for a duration (handy in ParallelAction while driving). */
    public Action keepUpdatingFor(double seconds) {
        return new TimedUpdater(seconds);
    }

    // --------------------- Internals / Utilities ---------------------

    /** Wait for a condition (done==true) or until timeout. Optionally run update loops while waiting. */
    public Action waitUntil(BooleanSupplier done, double timeoutSeconds, boolean runUpdates) {
        return new Action() {
            boolean started = false;
            long startNs;
            @Override public boolean run(TelemetryPacket p) {
                if (!started) { started = true; startNs = System.nanoTime(); }
                if (runUpdates) { shooter.update(); shooter.updateServo(); }
                double t = (System.nanoTime() - startNs) / 1e9;
                boolean timeLeft = t < timeoutSeconds;
                boolean finished = done.getAsBoolean();
                return !(finished || !timeLeft); // return true to KEEP running
            }
        };
    }

    /** Repeatedly calls update() / updateServo() for a fixed duration. */
    private class TimedUpdater implements Action {
        private final double seconds;
        private boolean started = false;
        private long startNs;
        TimedUpdater(double seconds) { this.seconds = seconds; }
        @Override public boolean run(TelemetryPacket p) {
            if (!started) { started = true; startNs = System.nanoTime(); }
            shooter.update();
            shooter.updateServo();
            double t = (System.nanoTime() - startNs) / 1e9;
            return t < seconds; // keep running until time elapses
        }
    }
}
