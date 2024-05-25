package org.firstinspires.ftc.teamcode.Outtake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Core.HWMap;

public class SlideSubsystem extends SubsystemBase {
    private Motor linearSlidesRight1;
    private Motor linearSlidesLeft;
    private Motor linearSlidesRight2;
    private static final double SAFE_HEIGHT = 35;//In CM
    private static final double EXTEND_SAFE_HEIGHT = 28;//In CM
    private static final double FULLY_RETRACTED_HEIGHT = 0.0;
    private static final int OFFSET_INCREMENT_CM = 1;
    private static final int MAX_HEIGHT = 72;
    public static double Pe = 0.065, PBelowSafeHeight = 0.082, I = 0.06, D = 0.005, F = 0.0015;
    private final PIDFController pidfController;
    private final double[] indexes = {FULLY_RETRACTED_HEIGHT, FULLY_RETRACTED_HEIGHT, 6.6, 13.2, 19.8, 26.4, 31.9, 38.5, 45.1, 51.7, 58.3, 64.9, 71.5, MAX_HEIGHT};
    //Each increment is 6.6 CM.
    private double targetPosition;
    private double measuredPosition;
    private SlideMotorsWrapper slideMotorsWrapper;
    private int currentIndex;
    private int currentOffset;
    private boolean slowMovement;
    private double slidePowerCap = 1.0;
    private final double slideFinalMovementsCap = 1.0;
    private final double slideMovementCap = 1.0;
    private double power = 0;

    public SlideSubsystem(final HWMap hwMap) {
        linearSlidesLeft = hwMap.getLinearSlidesLeft();
        linearSlidesRight1 = hwMap.getLinearSlidesRight1();
        linearSlidesRight2 = hwMap.getLinearSlidesRight2();
        slideMotorsWrapper = new SlideMotorsWrapper(hwMap);
        pidfController = new PIDFController(Pe, I, D, F);
    }

    /**
     * Description: This method increases the index by 1
     * Parameters: None
     */
    public void indexUp() {
        int tempIndex = currentIndex + 1;
        if (tempIndex < indexes.length) {
            currentIndex++;
        }
    }

    /**
     * Description: This method decreases the index by 1
     * Parameters: None
     */
    public void indexDown() {
        int tempIndex = currentIndex - 1;
        if (tempIndex >= 0) {
            currentIndex--;
        }
    }
    /**
     * Description: This method sets the target position of the slides to the current index ad offset position
     * Parameters: None
     */
    public void moveToSelectedIndexPosition() {
        //This method indexes and offsets the slides up and down so we can make mosaics
        targetPosition = indexes[currentIndex] + currentOffset;
    }
    public void updatePIDF() {
        if (targetPosition <= SAFE_HEIGHT)
            pidfController.setPIDF(PBelowSafeHeight, I, D, F);
        else
            pidfController.setPIDF(Pe, I, D, F);
        //This function will actually figure out the power needed and move the slides
        measuredPosition = slideMotorsWrapper.readPositionInCM();
        //The slides will move to target position only when the slides are extending.
        power = pidfController.calculate(measuredPosition, targetPosition);
        power = Math.min(Math.abs(power), Math.abs(slidePowerCap)) * Math.signum(power);

        if (!slowMovement)
            slideMotorsWrapper.set(power);
        else
            slideMotorsWrapper.set(-0.5);
    }
}
