package org.oastem.frc.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import java.util.Hashtable;




public class DriveSystemAccel extends DriveSystem{
	
	private long currTime;
	private long thisTime;
	//private double[] speed;
	private int[] locs;
	private Accelerator[] acceleration;
	private int locCount = 4;
	
	private DriveSystem(){
		currTime = System.currentTimeMillis();
		thisTime = currTime;
		acceleration = new Accelerator[12];
		speed = new double[12];
		locs = new int[12];
		super();
	}
	
	public static DriveSystem getInstance() {
        return super.getInstance;
    }
    
    public void initializeDrive(int leftFront, int leftRear, int rightFront, int rightRear) {
		locs[0] = leftFront;
		//locs[1] = leftRear;
		locs[2] = rightFront;
		//locs[3] = rightRear;
		acceleration[leftFront] = new Accelerator();
		//acceleration[leftRear] = new Accelerator();
		acceleration[rightFront] = new Accelerator();
		//acceleration[rightRear] = new Accelerator();
        super.initializeDrive(leftFront, leftRear, rightFront, rightRear);
    }
    
    public void initializeSecondaryDrive(int l2, int r2) {
		locs[locCount++] = l2;
		acceleration[l2] = new Accelerator();
		locs[locCount++] = r2;
		acceleration[r2] = new Accelerator();
        super.initializeSecondaryDrive(l2, r2);
    }
    
    public void addVictor(int port) {
		locs[locCount++] = port;
		//acceleration[port] = new Accelerator();
        super.addVictor(port);
    }
    
    public void set(int vic, double power) {
		speed[locs[vic]] = power;
        super.set(vic,power);
    }
    
    public void tankDrive(double x, double y) {
		x = acceleration[locs[0]].accelerateValue(x);
		y = acceleration[locs[2]].accelerateValue(y);
		super.tankDrive(x,y);
    }
    
    public double getAccelSpeed(int port){
		return acceleration[locs[port]].getSpeed();
	}
}
