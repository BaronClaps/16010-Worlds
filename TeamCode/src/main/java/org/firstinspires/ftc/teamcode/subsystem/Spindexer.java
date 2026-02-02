package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Artifact;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Spindexer {
    private static final int PHYSICAL_SLOTS = 3;
    private static final int THEORETICAL_POSITIONS = 5;
    public static boolean sort = false, autoRotate = false;
    public static int checkInterval = 2;
    public static double timeToSpin = 0.3, distThres = 2.0;
    private Timer spinTimer = new Timer();
    private Servo left, right;
    private final Servo k, b, t;
    public static double engage = 0.3, disengage = 0.1;
    public static double topGateOpen = 0.7, topGateClosed = 0.3;
    public static double bottomGateOpen = 0.2, bottomGateClosed = 0.7;
    private int loops;
    private RevColorSensorV3 sensor;

    public enum Index {
        ZERO(0.0),
        ONE(0.2),
        TWO(0.4),
        THREE(0.6),
        FOUR(0.8);

        private final double position;

        Index(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }

        public static Index fromInt(int index) {
            switch (index) {
                case 0:
                    return ZERO;
                case 1:
                    return ONE;
                case 2:
                    return TWO;
                case 3:
                    return THREE;
                case 4:
                    return FOUR;
                default:
                    throw new IllegalArgumentException("Invalid index: " + index);
            }
        }
    }

    // Track which slots have artifacts and their colors
    private Artifact[] slots;
    private int currentIndex;
    private Pattern currentPattern;
    private int shootDirection = 1; // Direction determined by optimal()

    public Spindexer(HardwareMap h) {
        this.slots = new Artifact[PHYSICAL_SLOTS];
        this.currentIndex = 0;
        this.currentPattern = null;
        this.shootDirection = 1;

        left = h.get(Servo.class, "spl");
        right = h.get(Servo.class, "spr");
        sensor = h.get(RevColorSensorV3.class, "spcolor");
        k = h.get(Servo.class, "k");
        b = h.get(Servo.class, "b");
        t = h.get(Servo.class, "t");
    }

    public void setPattern(Pattern pattern) {
        this.currentPattern = pattern;
    }

    public void add(Artifact color) {
        int physicalSlot = currentIndex % PHYSICAL_SLOTS;
        slots[physicalSlot] = color;
    }

    public void optimal() {
        if (currentPattern == null) {
            return;
        }

        Artifact[] patternColors = getPatternColors(currentPattern);

        int bestScore = -1;
        int bestStartIndex = -1;
        boolean bestForward = true;

        for (int startIdx = 0; startIdx < THEORETICAL_POSITIONS; startIdx++) {
            if (startIdx + 2 < THEORETICAL_POSITIONS) {
                int forwardScore = scorePattern(startIdx, true, patternColors);
                if (forwardScore > bestScore) {
                    bestScore = forwardScore;
                    bestStartIndex = startIdx;
                    bestForward = true;
                }
            }

            if (startIdx - 2 >= 0) {
                int backwardScore = scorePattern(startIdx, false, patternColors);
                if (backwardScore > bestScore) {
                    bestScore = backwardScore;
                    bestStartIndex = startIdx;
                    bestForward = false;
                }
            }
        }

        if (bestStartIndex != -1) {
            moveTo(bestStartIndex);
            shootDirection = bestForward ? 1 : -1;
        }
    }

    private int scorePattern(int startIdx, boolean forward, Artifact[] patternColors) {
        int score = 0;
        for (int i = 0; i < 3; i++) {
            int checkIdx = forward ? startIdx + i : startIdx - i;
            int physicalSlot = checkIdx % PHYSICAL_SLOTS;

            if (patternColors[i] == slots[physicalSlot]) {
                score++;
            }
        }
        return score;
    }

    private Artifact[] getPatternColors(Pattern pattern) {
        switch (pattern) {
            case GPP:
                return new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
            case PGP:
                return new Artifact[]{Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE};
            case PPG:
                return new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN};
            default:
                return new Artifact[3];
        }
    }

    public void empty() {
        int nearestEmpty = findNearestEmpty();
        if (nearestEmpty != -1) {
            moveTo(nearestEmpty);
        }
    }

    private int findNearestEmpty() {
        int shortestDistance = Integer.MAX_VALUE;
        int bestIndex = -1;

        for (int i = 1; i < THEORETICAL_POSITIONS; i++) {
            int forwardIdx = (currentIndex + i) % THEORETICAL_POSITIONS;
            int forwardSlot = forwardIdx % PHYSICAL_SLOTS;
            if (slots[forwardSlot] == null && i < shortestDistance) {
                shortestDistance = i;
                bestIndex = forwardIdx;
            }

            int backwardIdx = (currentIndex - i + THEORETICAL_POSITIONS) % THEORETICAL_POSITIONS;
            int backwardSlot = backwardIdx % PHYSICAL_SLOTS;
            if (slots[backwardSlot] == null && i < shortestDistance) {
                shortestDistance = i;
                bestIndex = backwardIdx;
            }
        }

        return bestIndex;
    }

    public void spin(int steps) {
        for (int i = 0; i < Math.abs(steps); i++) {
            if (steps > 0) {
                increment();
            } else {
                decrement();
            }
        }
    }

    private void set(double p) {
        left.setPosition(p);
        right.setPosition(p);
    }

    /**
     * Increments the spindexer by one index forward
     * Sets the hardware position based on the Index enum
     */
    private void increment() {
        currentIndex = (currentIndex + 1) % THEORETICAL_POSITIONS;
        Index index = Index.fromInt(currentIndex);
        set(index.get());
    }

    /**
     * Decrements the spindexer by one index backward
     * Sets the hardware position based on the Index enum
     */
    private void decrement() {
        currentIndex = (currentIndex - 1 + THEORETICAL_POSITIONS) % THEORETICAL_POSITIONS;
        Index index = Index.fromInt(currentIndex);
        set(index.get());
    }

    /**
     * Moves to a specific index (choosing shortest path)
     */
    private void moveTo(int targetIndex) {
        int forwardDist = (targetIndex - currentIndex + THEORETICAL_POSITIONS) % THEORETICAL_POSITIONS;
        int backwardDist = (currentIndex - targetIndex + THEORETICAL_POSITIONS) % THEORETICAL_POSITIONS;

        if (forwardDist <= backwardDist) {
            spin(forwardDist);
        } else {
            spin(-backwardDist);
        }
    }

    /**
     * Removes the artifact at the current position (after shooting)
     */
    public void remove() {
        int physicalSlot = currentIndex % PHYSICAL_SLOTS;
        slots[physicalSlot] = null;
    }

    /**
     * Gets the current theoretical index
     */
    public int currentIndex() {
        return currentIndex;
    }

    /**
     * Gets the current physical slot
     */
    public int currentPhysicalSlot() {
        return currentIndex % PHYSICAL_SLOTS;
    }

    /**
     * Checks if current slot has an artifact
     */
    public boolean hasArtifactAtCurrent() {
        int physicalSlot = currentIndex % PHYSICAL_SLOTS;
        return slots[physicalSlot] != null;
    }

    /**
     * Gets the color at current position (null if empty)
     */
    public Artifact getCurrentColor() {
        int physicalSlot = currentIndex % PHYSICAL_SLOTS;
        return slots[physicalSlot];
    }

    public double[] getColor() {
        double[] rgb = new double[3];
        if (sensor == null) return rgb;
        try {
            double r = sensor.red();
            double g = sensor.green();
            double b = sensor.blue();
            double max = Math.max(r, Math.max(g, b));
            if (max > 0.0) {
                rgb[0] = r / max;
                rgb[1] = g / max;
                rgb[2] = b / max;
            } else {
                rgb[0] = 0.0;
                rgb[1] = 0.0;
                rgb[2] = 0.0;
            }
        } catch (Exception ignored) {
        }
        return rgb;
    }

    public Artifact getClosestColor() {
        double[] sample = getColor();
        double r = sample[0];
        double g = sample[1];
        double b = sample[2];

        double maxChannel = Math.max(r, Math.max(g, b));
        final double NO_READ_THRESHOLD = 1e-3;
        if (maxChannel < NO_READ_THRESHOLD) {
            return Artifact.UNIDENTIFIED;
        }

        double bestDist = Double.POSITIVE_INFINITY;
        Artifact best = Artifact.UNIDENTIFIED;
        for (Artifact c : Artifact.values()) {
            if (c == Artifact.UNIDENTIFIED) continue;
            double dr = c.r - r;
            double dg = c.g - g;
            double db = c.b - b;
            double dist = dr * dr + dg * dg + db * db;
            if (dist < bestDist) {
                bestDist = dist;
                best = c;
            }
        }

        return best;
    }

    public int isShootForward() {
        return shootDirection;
    }

    public void reset() {
        slots = new Artifact[PHYSICAL_SLOTS];
        currentIndex = 0;
        currentPattern = null;
        shootDirection = 1;
    }

    public boolean full() {
        int filled = 0;
        for (Artifact slot : slots) {
            if (slot != null) {
                filled++;
            }
        }
        return filled >= PHYSICAL_SLOTS;
    }

    public void all() {
        int direction = isShootForward();
        for (int i = 0; i < PHYSICAL_SLOTS; i++) {
            remove();

            if (i < 2) {
                spin(direction);
            }
        }
    }

    public void engageKicker() {
        k.setPosition(engage);
    }

    public void disengageKicker() {
        k.setPosition(disengage);
    }

    public void openTopGate() {
        t.setPosition(topGateOpen);
    }

    public void closeTopGate() {
        t.setPosition(topGateClosed);
    }

    public void openBottomGate() {
        b.setPosition(bottomGateOpen);
    }

    public void closeBottomGate() {
        b.setPosition(bottomGateClosed);
    }

    public void enableSort() {
        sort = true;
        autoRotate = true;
    }

    public void disableSort() {
        sort = false;
    }

    public void enableAutoRotate() {
        autoRotate = true;
    }

    public void disableAutoRotate() {
        autoRotate = false;
    }

    public void enablePassthrough() {
        sort = false;
        autoRotate = false;
    }

    public void periodic() {
        if (autoRotate) {
            loops++;
            if (loops % checkInterval == 0 && !full()) {
                if (spinTimer.getElapsedTimeSeconds() >= timeToSpin) {
                    double distanceInches = sensor.getDistance(DistanceUnit.INCH);
                    if (!Double.isNaN(distanceInches) && distanceInches > distThres) {
                        Artifact detected = Artifact.UNIDENTIFIED;
                        if (sort)
                            detected = getClosestColor();

                        add(detected);
                        empty();

                        spinTimer.resetTimer();

                        if (full())
                            optimal();
                    }
                }
            }
        }
    }
}
