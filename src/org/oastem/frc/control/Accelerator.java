package org.oastem.frc.control;

/**
 * @author ThePotatoGuy
 */
public class Accelerator {

    private long currTime;
    private long thisTime;
    private double speed;
    public static final double THRESHOLD = 0.005;
    public static final double ACCEL_FACTOR = 0.02;

    public Accelerator() {
        currTime = System.currentTimeMillis();
        thisTime = currTime;
        speed = 0.0;
    }

    public double accelerateValue(double commSpeed) {
        currTime = System.currentTimeMillis();
        speed = accelerate(speed, commSpeed, (currTime - thisTime));
        return speed;
    }

    private double accelerate(double currSpeed, double commandSpeed, long time) {
        if (Math.abs(currSpeed - commandSpeed) > THRESHOLD) {
            if (time > 40L) {
                if (commandSpeed > currSpeed) {
                    currSpeed = currSpeed + ACCEL_FACTOR;
                } else if (commandSpeed < currSpeed) {
                    currSpeed = currSpeed - ACCEL_FACTOR;
                }
                thisTime = currTime;
            }
            return currSpeed;
        }
        return commandSpeed;
    }

    public double getSpeed() {
        return speed;
    }
}
