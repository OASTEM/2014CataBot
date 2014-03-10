package org.oastem.frc.control;

public class Accelerator{
	
	private long currTime;
	private long thisTime;
	private double speed;
	
	public Accelerator(){
		currTime = System.currentTimeMillis();
		thisTime = currTime;
		speed = 0.0;
	}
	
	public double accelerateValue(double commSpeed){
		currTime = System.currentTimeMillis();
		speed = accelerate(speed, commSpeed, (currTime-thisTime));
		return speed;
	}
	
	private double accelerate(double currSpeed, double commandSpeed, long time){
        if(Math.abs(currSpeed - commandSpeed) > THRESHOLD && time > 50L){
			if(commandSpeed > currSpeed){
				currSpeed = currSpeed + ACCEL_FACTOR;
			}
			else if(commandSpeed < currSpeed){
				currSpeed = currSpeed - ACCEL_FACTOR;
			}
			theTime = currTime;
			return currSpeed;
		}
        return commandSpeed;
    }
    
    public double getSpeed(){
		return speed;
	}
}
