package org.oastem.frc.control;

/**
 * @author ThePotatoGuy
 */
public class Accelerator {

    private long currTime;
    private long thisTime;
    private double speed;
    public static final double THRESHOLD = 0.005;
    public static final double ACCEL_FACTOR = 0.2;
    //private boolean skipper;

    public Accelerator() {
        //skipper = false;
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
        System.out.println(currSpeed+" Distance/Time");
        if (Math.abs(currSpeed - commandSpeed) > THRESHOLD) {
           if (time > 40L) {
                if (commandSpeed > currSpeed) {
                    // acceleration
                    currSpeed = currSpeed + ACCEL_FACTOR;
                } else if (commandSpeed < currSpeed) {
                    // deceleration
                    currSpeed = currSpeed - ACCEL_FACTOR/2;
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
