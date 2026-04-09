package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class ShooterActions {

    private final ShooterSystem shooter;
    private int lastAutonShootIndex = -1;

    public ShooterActions(ShooterSystem shooter) {
        this.shooter = shooter;
    }

    // =========================================================
    // CORE UPDATE
    // =========================================================
    private void updateAll() {
        shooter.updateShooterBangBang();
        shooter.updateAngler();
        shooter.updateSpindexer();
    }

    private void updateEverything() {
        shooter.update();
    }

    // =========================================================
    // SIMPLE SETTERS
    // =========================================================
    public Action setShooterRpm(double rpm) {
        return p -> {
            shooter.setShooterTargetRpm(rpm);
            return false;
        };
    }

    public Action setHoodDeg(double pos) {
        return p -> {
            shooter.setHoodPosition(pos);
            return false;
        };
    }

    // =========================================================
    // KICKER
    // =========================================================
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

    // =========================================================
    // LIMELIGHT
    // =========================================================
    public AutonColorManager.BallColor[] readMotifFromLimelight(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
            switch (fid.getFiducialId()) {
                case 21:
                    return new AutonColorManager.BallColor[]{
                            AutonColorManager.BallColor.GREEN,
                            AutonColorManager.BallColor.PURPLE,
                            AutonColorManager.BallColor.PURPLE
                    };
                case 22:
                    return new AutonColorManager.BallColor[]{
                            AutonColorManager.BallColor.PURPLE,
                            AutonColorManager.BallColor.GREEN,
                            AutonColorManager.BallColor.PURPLE
                    };
                case 23:
                    return new AutonColorManager.BallColor[]{
                            AutonColorManager.BallColor.PURPLE,
                            AutonColorManager.BallColor.PURPLE,
                            AutonColorManager.BallColor.GREEN
                    };
            }
        }
        return null;
    }

    // =========================================================
    // AUTON SHOOT SEQUENCE (SAFE)
    // =========================================================
    public Action autonMoveToNextShootIndex(AutonColorManager mgr) {
        return p -> {
            int shootIdx = mgr.getNextShootIndex();
            if (shootIdx == -1) return false;

            lastAutonShootIndex = shootIdx;
            shooter.setShootIndex(shootIdx);
            updateAll();
            return false;
        };
    }

    public Action autonConfirmShot(AutonColorManager mgr) {
        return p -> {
            if (lastAutonShootIndex != -1) {
                mgr.onShot(lastAutonShootIndex);
                lastAutonShootIndex = -1;
            }
            return false;
        };
    }

    // =========================================================
    // INTAKE
    // =========================================================
    public Action intakeForward(double power) {
        return p -> {
            shooter.intakeOn(power);
            return false;
        };
    }

    public Action intakeReverse(double power) {
        return p -> {
            shooter.intakeReverse(power);
            return false;
        };
    }

    public Action autonRegisterIntake(
            AutonColorManager mgr,
            AutonColorManager.BallColor color
    ) {
        return p -> {
            mgr.onIntake(shooter.intakeIndex, color);
            return false;
        };
    }

    // =========================================================
    // INDEXING
    // =========================================================
    public Action indexNextIntakeSlot(double unused) {
        return p -> {
            shooter.KickerDown();
            shooter.stepIntakeSlot();
            updateAll();
            return false;
        };
    }
    // =========================================================
// HUSKY SERVO ACTIONS
// =========================================================

    public Action huskyUp() {
        return p -> {
            shooter.huskyUp();
            return false;
        };
    }

    public Action huskyDown() {
        return p -> {
            shooter.huskyDown();
            return false;
        };
    }


    public Action indexNextShootSlot(double unused) {
        return p -> {
            shooter.KickerDown();
            shooter.stepShootSlot();
            updateAll();
            return false;
        };
    }

    public Action setIntakeIndex(int idx) {
        return p -> {
            shooter.setIntakeIndex(idx);
            updateAll();
            return false;
        };
    }

    public Action setShootIndex(int idx) {
        return p -> {
            shooter.setShootIndex(idx);
            updateAll();
            return false;
        };
    }

    // =========================================================
    // WAIT HELPERS
    // =========================================================
    public Action keepUpdatingFor(double seconds) {
        return new Action() {
            long start = -1;
            @Override
            public boolean run(TelemetryPacket p) {
                if (start < 0) start = System.nanoTime();
                updateAll();
                return (System.nanoTime() - start) / 1e9 < seconds;
            }
        };
    }

    public Action waitUntil(BooleanSupplier done, double timeout, boolean update) {
        return new Action() {
            long start = -1;
            @Override
            public boolean run(TelemetryPacket p) {
                if (start < 0) start = System.nanoTime();
                if (update) updateAll();
                return !done.getAsBoolean() &&
                        (System.nanoTime() - start) / 1e9 < timeout;
            }
        };
    }

    // =========================================================
// COLOR SORTING / HUSKY
// =========================================================

    public Action updateColorDetection() {
        return p -> {
            shooter.updateColorDetection();
            return false;
        };
    }

    public Action waitForBallDetection(double timeoutSeconds) {
        return new Action() {
            long start = -1;

            @Override
            public boolean run(TelemetryPacket p) {

                if(start < 0) start = System.nanoTime();

                shooter.updateColorDetection();

                boolean ballSeen = shooter.ballCurrentlyDetected;

                return !ballSeen &&
                        (System.nanoTime() - start) / 1e9 < timeoutSeconds;
            }
        };
    }

    public Action waitForSpecificColor(int colorID, double timeoutSeconds) {
        return new Action() {
            long start = -1;

            @Override
            public boolean run(TelemetryPacket p) {

                if(start < 0) start = System.nanoTime();

                shooter.updateColorDetection();

                boolean match =
                        shooter.ballCurrentlyDetected &&
                                shooter.detectedColor == colorID;

                return !match &&
                        (System.nanoTime() - start) / 1e9 < timeoutSeconds;
            }
        };
    }

    // =========================================================
// AUTO INDEXING
// =========================================================

    public Action runAutoIndexing() {
        return p -> {
            shooter.updateColorDetection();
            return true; // keep running
        };
    }

    public Action runAutoIndexingFor(double seconds) {
        return new Action() {

            long start = -1;

            @Override
            public boolean run(TelemetryPacket p) {

                if(start < 0) start = System.nanoTime();

                shooter.updateColorDetection();

                return (System.nanoTime() - start) / 1e9 < seconds;
            }
        };
    }
}
