package org.firstinspires.ftc.teamcode.Outtake;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.Core.HWMap;


public class SlideMotorsWrapper {
    private final Motor linearSlidesLeft;
    private final Motor linearSlidesRight1;
    private final Motor linearSlidesRight2;
    private double lastReadPositionInCM;
    private static double SLIDES_CPR;

    public SlideMotorsWrapper(HWMap hwMap){
        linearSlidesLeft = hwMap.getLinearSlidesLeft();
        linearSlidesRight1 = hwMap.getLinearSlidesRight1();
        linearSlidesRight2 = hwMap.getLinearSlidesRight2();
        SLIDES_CPR = linearSlidesLeft.getCPR();
    }
    /**
     * Description: This method sets power to the linear slides motors
     * Parameters: power (-1:1)
     */
    public void set(double power){
        linearSlidesLeft.set(power);
        linearSlidesRight1.set(power);
        linearSlidesRight2.set(power);
    }
    /**
     * Description: This method reads and returns the position in CM of the linear slides (The most current value).
     * Parameters: None
     */
    public double readPositionInCM(){
        double currentPositionInTicks = linearSlidesLeft.getCurrentPosition();
        double diameterOfSpool = 4.6;
        double ratio = (80.0 / 20) * (38.0 / 16);
        lastReadPositionInCM = (currentPositionInTicks / (SLIDES_CPR * ratio)) * Math.PI * diameterOfSpool;
        return lastReadPositionInCM;
    }
    public double getCurrentVelocity(){
        double currentVelocity = linearSlidesLeft.getCorrectedVelocity();
        double diameterOfSpool = 4.6;
        double ratio = (80.0 / 20) * (38.0 / 16);
        return (currentVelocity / (SLIDES_CPR * ratio)) * Math.PI * diameterOfSpool;
    }
    /**
     * Description: This method returns the value that was last read from the slides (Not to most current value).
     * Parameters: None
     */
    public double getLastReadPositionInCM() {
        return lastReadPositionInCM;
    }
}
