package org.oastem.frc.control;

/**
 * @author ThePotatoGuy
 * updated 2016-09-25 with comments
 * 
 * Each accelerator knows its own speed and a couple of important factors
 * This class allows motors to accelerate at a gradual (or not) pace
 * 
 * Acceleration only occurs when:
 *  1 - the desired speed is signifcantly larger/smaller than the 
 *      current speed (elimnates changes in speed that were NOT 
 *      deliberate)
 *  2 - an appropriate amount of time has passed since the last
 *      acceleration (allows for gradual acceleration)
 * 
 * NOTES:
 * commSpeed -> commanded speed -> the speed the drive desires
 * speed -> the current speed of this accelerator
 */
public class Accelerator {

    private long currTime; // current time of this Accelerator
    private long thisTime; // time of last successful acceleration
    private double speed;
    
    // limit to remove unwanted acceleration from tiny changes in
    // speed that were not from teh control
    // lower threshold means high sensitivy from control (joystick)
    public static final double THRESHOLD = 0.005;
    
    // some factor to increase/decrease the speeds by
    public static final double ACCEL_FACTOR = 0.2;
    public static final double DECEL_FACTOR = 0.1;
    
    // delay in miliseconds to accelerate
    // lower delay means more instantaneous acceleration
    public static final long ACCEL_DELAY = 40L;

    public Accelerator() {
        //skipper = false;
        currTime = System.currentTimeMillis();
        thisTime = currTime;
        speed = 0.0;
    }

    /**
     * accelerates this Accelerator to the given commSpeed
     * (really only an attempt to do so, read comment on accelerate())
     * 
     * passes this Accelerator's current speed, the commanded speed,
     * and difference in time from the last time this method was called
     * to the accelerate() function
     * 
     * IN:
     *  @param commSpeed    - the speed we desire this Accelerator to 
     *      be at
     * 
     * OUT:
     *  @return the current speed value of this Accelerator 
     *      post-acceleration (it might not have changed)
     */
    public double accelerateValue(double commSpeed) {
        currTime = System.currentTimeMillis();
        speed = accelerate(speed, commSpeed, (currTime - thisTime));
        return speed;
    }

    /**
     * Does the actual acceleratation if certain conditions are met:
     * 
     * Basic overview:
     * If the difference between the commanded speed and the current 
     * speed is greater than the given threshold (ie: drive system is
     * saying to accelerate this Accelerator), check if enough time has
     * passed between the last call to accelerate and this one. (this 
     * is to keep the acceleration gradual, like real acceleration)
     * If enough time has passed, increment/decrement the current speed
     * according to whether or not we are positively or negatively
     * accelerating. Then set the last called time to the currently 
     * called time so the next call will be gradual as well
     * 
     * IN:
     *  @param currSpeed    - the current speed we are at
     *  @param commandSpeed - the speed we wish to be at
     *  @param time         - the amount of time passed between now
     *      and last successful acceleration
     * 
     * OUT:
     *  @return the current speed of this Accelerator post-acceleration
     *      (might not have changed)
     */
    private double accelerate(double currSpeed, double commandSpeed, long time) {
        System.out.println(currSpeed+" Distance/Time");
        if (Math.abs(currSpeed - commandSpeed) > THRESHOLD) {
            /* the desired speed is ACTUALLY desired and not
             * flucuations in the joystick (drive system)
             */
            
           if (time > ACCEL_DELAY) {
               /* enough time has passed so we can accelerate
                * (for gradual acceleration)
                */
               
                if (commandSpeed > currSpeed) {
                    // acceleration (basic incremnt by)
                    currSpeed += ACCEL_FACTOR;
                } else if (commandSpeed < currSpeed) {
                    // deceleration (basic decremnt by)
                    currSpeed -= DECEL_FACTOR;
                }
                
                // this is now the last time we successfully accelerated
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
