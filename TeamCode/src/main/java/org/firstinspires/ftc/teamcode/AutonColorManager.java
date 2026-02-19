package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

public class AutonColorManager {

    public enum BallColor {
        GREEN,
        PURPLE,
        EMPTY
    }

    // Logical intake slots (0–2)
    private final BallColor[] spindexer = {
            BallColor.EMPTY,
            BallColor.EMPTY,
            BallColor.EMPTY
    };

    // Desired shooting order (length ≤ 3)
    private BallColor[] motif = null;
    private int motifIndex = 0;

    // -------------------------------------------------
    // Mapping: INTAKE slot → SHOOT slot
    // Intake 0 -> Shoot 1
    // Intake 1 -> Shoot 2
    // Intake 2 -> Shoot 0
    // -------------------------------------------------
    private static final int[] INTAKE_TO_SHOOT = {1, 2, 0};

    private static final int[] SHOOT_TO_INTAKE = {2, 0, 1};

    // -------------------------------------------------
    // Motif
    // -------------------------------------------------
    public void setMotif(BallColor[] motifPattern) {
        this.motif = motifPattern;
        this.motifIndex = 0;
    }

    // resets ONLY shooting order progress
    public void resetMotifIndex() {
        this.motifIndex = 0;
    }

    // clears physical state (ONLY before intake)
    public void clearSpindexer() {
        for (int i = 0; i < spindexer.length; i++) {
            spindexer[i] = BallColor.EMPTY;
        }
        motifIndex = 0;
    }


    // -------------------------------------------------
    // Intake registration
    // -------------------------------------------------
    public void onIntake(int intakeIndex, BallColor color) {
        spindexer[Math.floorMod(intakeIndex, 3)] = color;
    }

    // -------------------------------------------------
    // Decide next SHOOT index (ABSOLUTE)
    // -------------------------------------------------
    public int getNextShootIndex() {
        if (motif == null || motifIndex >= motif.length) {
            return findAnyShootIndex();
        }

        BallColor desired = motif[motifIndex];

        for (int intakeIdx = 0; intakeIdx < 3; intakeIdx++) {
            if (spindexer[intakeIdx] == desired) {
                return INTAKE_TO_SHOOT[intakeIdx];
            }
        }

        // Fallback: shoot anything
        return findAnyShootIndex();
    }

    private int findAnyShootIndex() {
        for (int intakeIdx = 0; intakeIdx < 3; intakeIdx++) {
            if (spindexer[intakeIdx] != BallColor.EMPTY) {
                return INTAKE_TO_SHOOT[intakeIdx];
            }
        }
        return -1;
    }

    // -------------------------------------------------
    // Shot confirmation (ADVANCES MOTIF)
    // -------------------------------------------------
    public void onShot(int shootIndex) {
        int intakeIdx = SHOOT_TO_INTAKE[Math.floorMod(shootIndex, 3)];
        spindexer[intakeIdx] = BallColor.EMPTY;
        motifIndex++;
    }

    // -------------------------------------------------
    // Debug
    // -------------------------------------------------
    @Override
    public String toString() {
        return "Slots=" + Arrays.toString(spindexer) +
                " motifIndex=" + motifIndex;
    }
}
