package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Road Runner Actions wrapper for your ShooterSystem.
 *
 * IMPORTANT:
 * - Any Action that "waits" MUST keep calling shooter.updateShooterBangBang(),
 *   shooter.updateAngler(), shooter.updateSpindexer() so controllers keep running.
 * - Your ShooterSystem currently doesn't maintain a real "spinningServo" flag for the spindexer,
 *   so index waits are implemented by checking encoder error to targetRad.
 */
public class ShooterActions {

    private final ShooterSystem shooter;
    private final int[] spindexerBalls;

    // ---- tunables for waits ----
    private static final double SPINDEXER_TOL_RAD = Math.toRadians(2.0); // ~2 deg
    private static final double HOOD_TOL_DEG      = 1.5;                // hood settle tolerance (optional)

    public ShooterActions(ShooterSystem shooter, int[] spindexerBalls) {
        this.shooter = shooter;
        this.spindexerBalls = spindexerBalls;
    }

    // =========================================================
    //                 "UPDATE EVERYTHING" HELPER
    // =========================================================
    private void updateAll() {
        // Keep these running constantly so targets are held.
        shooter.updateShooterBangBang();
        shooter.updateAngler();
        shooter.updateSpindexer();
    }

    // =========================================================
    //                 INSTANT / SETTER ACTIONS
    // =========================================================
    public Action setShooterMode(ShooterSystem.ShooterControlMode mode) {
        return p -> {
            shooter.setShooterMode(mode);
            return false;
        };
    }

    public Action setShooterRpm(double rpm) {
        return p -> {
            shooter.setShooterTargetRpm(rpm);
            return false;
        };
    }

    public Action stopShooter() {
        return p -> {
            shooter.stopShooterBangBang();
            return false;
        };
    }

    public Action setHoodDeg(double hoodDeg) {
        return p -> {
            shooter.setTargetangler(hoodDeg);
            return false;
        };
    }

    public Action kickerUp() {
        return p -> {
            shooter.KickerUp();
            return false;
        };
    }

    public Action kickerDown() {
        return p -> {
            shooter.KickerDown();
            return false;
        };
    }

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

    // =========================================================
    //                 SIMPLE COMPOSED ACTIONS
    // =========================================================

    /** Kicker up, wait, kicker down (non-blocking to RR, but still updates controllers during wait). */
    public Action kickOnce(double upSeconds) {
        return new SequentialAction(
                kickerUp(),
                keepUpdatingFor(upSeconds),
                kickerDown()
        );
    }

    /** Run feeder full forward for N seconds then stop (still updates during the time). */
    public Action feedFull(double seconds) {
        return new SequentialAction(
                p -> { shooter.controlFeeder(1, 0); return false; },
                keepUpdatingFor(seconds),
                p -> { shooter.controlFeeder(0, 0); return false; }
        );
    }

    // =========================================================
    //                 SPINDEXER INDEX ACTIONS
    // =========================================================

    /** Step to next INTAKE slot and wait until spindexer reaches the target (encoder-based). */
    public Action indexNextIntakeSlot(double timeoutSeconds) {
        return new SequentialAction(
                p -> { shooter.KickerDown(); shooter.nextIntakeSlot(); return false; },
                waitUntil(this::spindexerAtTarget, timeoutSeconds, true)
        );
    }

    /** Step to next SHOOT slot and wait until spindexer reaches the target (encoder-based). */
    public Action indexNextShootSlot(double timeoutSeconds) {
        return new SequentialAction(
                p -> { shooter.KickerDown(); shooter.nextShootSlot(); return false; },
                waitUntil(this::spindexerAtTarget, timeoutSeconds, true)
        );
    }

    private boolean spindexerAtTarget() {
        // spindexerEncoder is in radians, targetRad is radians
        double cur = shooter.spindexerEncoder.getCurrentPosition();
        double tgt = shooter.targetRad;
        double err = angleWrapRad(tgt - cur);
        return Math.abs(err) <= SPINDEXER_TOL_RAD;
    }

    private static double angleWrapRad(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // =========================================================
    //                 AUTO-SHOT (DISTANCE -> RPM + HOOD)
    // =========================================================

    /**
     * Runs ShooterSystem.updateAutoShot(distance) continuously for N seconds,
     * while also running all controllers (shooter + hood + spindexer).
     *
     * You pass a distance supplier (ex: from Limelight).
     */
    public Action autoShotFor(DoubleSupplier distanceInches, double seconds) {
        return new Action() {
            long startNs = -1;

            @Override
            public boolean run(TelemetryPacket p) {
                if (startNs == -1) startNs = System.nanoTime();

                double t = (System.nanoTime() - startNs) / 1e9;
                if (t >= seconds) return false;

                double d = distanceInches.getAsDouble();
                shooter.updateAutoShot(d);

                updateAll();
                return true;
            }
        };
    }

    /**
     * Wait until shooter RPM is within your ShooterSystem.atShooterSpeed()
     * (or timeout), while still updating everything.
     */
    public Action waitShooterAtSpeed(double timeoutSeconds) {
        return waitUntil(shooter::atShooterSpeed, timeoutSeconds, true);
    }

    // =========================================================
    //                 KEEP-UPDATING / WAIT HELPERS
    // =========================================================

    /** Keep updating all controllers for a duration. */
    public Action keepUpdatingFor(double seconds) {
        return new Action() {
            long startNs = -1;

            @Override
            public boolean run(TelemetryPacket p) {
                if (startNs == -1) startNs = System.nanoTime();

                double t = (System.nanoTime() - startNs) / 1e9;
                if (t >= seconds) return false;

                updateAll();
                return true;
            }
        };
    }

    /**
     * Wait until condition is true OR timeout occurs.
     * If doUpdates = true, keeps controllers running during the wait.
     */
    public Action waitUntil(BooleanSupplier done, double timeoutSeconds, boolean doUpdates) {
        return new Action() {
            long startNs = -1;

            @Override
            public boolean run(TelemetryPacket p) {
                if (startNs == -1) startNs = System.nanoTime();

                if (doUpdates) updateAll();

                double t = (System.nanoTime() - startNs) / 1e9;

                // finish if done OR timed out
                if (done.getAsBoolean() || t >= timeoutSeconds) return false;

                return true;
            }
        };
    }
}
