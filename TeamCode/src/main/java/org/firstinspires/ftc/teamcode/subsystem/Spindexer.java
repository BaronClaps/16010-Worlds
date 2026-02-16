package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Artifact;
import org.firstinspires.ftc.teamcode.util.Pattern;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Spindexer {
    private static final int PHYSICAL_SLOTS = 3;
    private static final int THEORETICAL_POSITIONS = 5;

    public static boolean sort = false, autoRotate = false, shooting = false;
    public static int checkInterval = 2;
    public static double timeToSpin = 0.3, timeToShoot = 0.7, upperDistThres = 1.6, lowerDistThres = 1.4, needToShoot = -1;

    public static double kEngaged = .85, kDisengaged = 1, bgOpen = 0.36, bgClosed = .5, tgOpen = .575, tgClosed = .675;
    public double dist;
    public Timer done = new Timer();
    private final Timer spinTimer = new Timer(), shootTimer = new Timer(), allTimer = new Timer();
    private Servo right, left, k, bg, tg; // Left/Kicker commented out as per original
    public final RevColorSensorV3 sensor;

    public enum Index {
        ZERO(0.15), ONE(0.325), TWO(0.525), THREE(0.7), FOUR(0.875);
        private final double position;

        Index(double position) {
            this.position = position;
        }

        public double get() {
            return position;
        }

        public static Index fromInt(int index) {
            return values()[Math.max(0, Math.min(index, 4))];
        }
    }

    public Artifact[] slots = new Artifact[PHYSICAL_SLOTS];
    public int currentIndex;
    private Pattern currentPattern = null;
    public int shootDirection = 1;
    private int loops;

    public Spindexer(HardwareMap h) {
        right = h.get(Servo.class, "spr");
        left = h.get(Servo.class, "spl");
        sensor = h.get(RevColorSensorV3.class, "c");
        k = h.get(Servo.class, "k");
        bg = h.get(Servo.class, "bg");
        tg = h.get(Servo.class, "tg");

        slots = new Artifact[PHYSICAL_SLOTS];
    }

    // --- LOGIC ---

    public void optimal() {
        if (currentPattern == null) return;
        Artifact[] goal = getPatternColors(currentPattern);

        int bestScore = -1;
        int bestIndex = currentIndex;
        int bestDir = 1;

        // Check current position and immediate neighbors to minimize travel
        int[] nearbyIndices = {currentIndex, currentIndex + 1, currentIndex - 1};

        for (int testIdx : nearbyIndices) {
            if (testIdx < 0 || testIdx >= THEORETICAL_POSITIONS) continue;

            // Check Forward path from this testIdx
            if (testIdx + 2 < THEORETICAL_POSITIONS) {
                int fScore = scoreSequence(testIdx, 1, goal);
                if (fScore > bestScore) {
                    bestScore = fScore;
                    bestIndex = testIdx;
                    bestDir = 1;
                }
            }

            // Check Backward path from this testIdx
            if (testIdx - 2 >= 0) {
                int bScore = scoreSequence(testIdx, -1, goal);
                if (bScore > bestScore) {
                    bestScore = bScore;
                    bestIndex = testIdx;
                    bestDir = -1;
                }
            }
            // If perfect match found at current or +1/-1, stop searching
            if (bestScore == 3) break;
        }

        moveTo(bestIndex);
        this.shootDirection = bestDir;
    }

    private int scoreSequence(int startIdx, int dir, Artifact[] goal) {
        int score = 0;
        for (int step = 0; step < 3; step++) {
            int physSlot = (startIdx + (step * dir) + PHYSICAL_SLOTS * 2) % PHYSICAL_SLOTS;
            if (slots[physSlot] == goal[step]) score++;
        }
        return score;
    }

    private Artifact[] getPatternColors(Pattern pattern) {
        if (pattern == Pattern.GPP) return new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
        if (pattern == Pattern.PGP) return new Artifact[]{Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE};
        return new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN}; // PPG
    }

    // --- MOVEMENT ---

    public void moveTo(int targetIndex) {
        currentIndex = Math.max(0, Math.min(targetIndex, THEORETICAL_POSITIONS - 1));
        double pos = Index.fromInt(currentIndex).get();
        right.setPosition(pos);
        left.setPosition(pos);
    }

    public void spin(int steps) {
        moveTo(currentIndex + steps);
    }

    public void all() {
        openTopGate();
        openBottomGate();
        engageKicker();
        shooting = true;
        shootTimer.resetTimer();
        allTimer.resetTimer();
        needToShoot = 3;
    }

    public void all(double needToShoot) {
        openTopGate();
        openBottomGate();
        engageKicker();
        shooting = true;
        shootTimer.resetTimer();
        allTimer.resetTimer();
        Spindexer.needToShoot = needToShoot;
    }

    // --- SUBSYSTEM TOOLS ---

    public void add(Artifact color) {
        slots[currentIndex % PHYSICAL_SLOTS] = color;
    }

    public void remove() {
        slots[currentIndex % PHYSICAL_SLOTS] = null;
    }

    public void empty() {
        int bestIdx = -1;
        int minSnaps = 10;
        for (int i = 0; i < THEORETICAL_POSITIONS; i++) {
            if (slots[i % PHYSICAL_SLOTS] == null) {
                int dist = Math.abs(i - currentIndex);
                if (dist < minSnaps) {
                    minSnaps = dist;
                    bestIdx = i;
                }
            }
        }
        if (bestIdx != -1) moveTo(bestIdx);
    }

    public Artifact getSimpleColor() {
        // Pull raw values directly from the sensor
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();

        // Check if there is enough light to make a reading
        double maxChannel = Math.max(r, Math.max(g, b));
        final double NO_READ_THRESHOLD = 1e-3;
        if (maxChannel < NO_READ_THRESHOLD) {
            return Artifact.UNIDENTIFIED;
        }

        double bestDist = Double.POSITIVE_INFINITY;
        Artifact best = Artifact.UNIDENTIFIED;

        // Loop through Artifact enum values to find the mathematical "nearest neighbor"
        for (Artifact c : Artifact.values()) {
            if (c == Artifact.UNIDENTIFIED) continue;

            double dr = c.r - r;
            double dg = c.g - g;
            double db = c.b - b;

            // Calculate squared Euclidean distance
            double dist = dr * dr + dg * dg + db * db;

            if (dist < bestDist) {
                bestDist = dist;
                best = c;
            }
        }

        // If the closest match is still too far away, don't trust the result
        if (bestDist > 0.25) {
            return Artifact.UNIDENTIFIED;
        }

        return best;
    }

    public boolean full() {
        for (Artifact a : slots) if (a == null) return false;
        return true;
    }

    public void reset() {
        slots = new Artifact[PHYSICAL_SLOTS];
        currentIndex = 2;
        moveTo(2);
    }

    public void periodic() {
        loops++;

        if (shootTimer.getElapsedTimeSeconds() > timeToShoot && shooting) {
            shooting = false;
            done.resetTimer();
        } else {
            if (needToShoot > 0 && allTimer.getElapsedTimeSeconds() > timeToShoot/4) {
                remove();
                spin(shootDirection);
                allTimer.resetTimer();
                needToShoot--;
            }
        }

        if (!autoRotate) return;

        if (loops % checkInterval == 0 && !full() && !shooting) {
            if (spinTimer.getElapsedTimeSeconds() >= timeToSpin) {
                dist = sensor.getDistance(DistanceUnit.INCH);
                if (dist < upperDistThres && dist > lowerDistThres) {
                    add(sort ? getSimpleColor() : Artifact.UNIDENTIFIED);
                    if (!full()) empty();
                    spinTimer.resetTimer();
                }
            }
        }

    }

    public Timer getShootTimer() {
        return shootTimer;
    }

    // Standard Toggle/Setters
    public void setPattern(Pattern p) {
        this.currentPattern = p;
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

    public void engageKicker() {
        k.setPosition(kEngaged);
    }

    public void disengageKicker() {
        k.setPosition(kDisengaged);
    }

    public void openTopGate() {
        tg.setPosition(tgOpen);
    }

    public void closeTopGate() {
        tg.setPosition(tgClosed);
    }

    public void openBottomGate() {
        bg.setPosition(bgOpen);
    }

    public void closeBottomGate() {
        bg.setPosition(bgClosed);
    }
}