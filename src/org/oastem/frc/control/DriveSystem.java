/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import java.util.Hashtable;

/**
 *
 * @author KTOmega
 */
public class DriveSystem {
    private static DriveSystem instance;
    private RobotDrive drive;
    private Victor[] raw;
    private boolean hasSecondary = false;
    private RobotDrive drive2;
    
    private DriveSystem() {
        raw = new Victor[12];
    }
    
    public static DriveSystem getInstance() {
        if (instance == null) {
            instance = new DriveSystem();
        }
        
        return instance;
    }
    
    public void initializeDrive(int leftFront, int leftRear, int rightFront, int rightRear) {
        drive = new RobotDrive(leftFront, leftRear, rightFront, rightRear);
    }
    
    public void setDrive(RobotDrive rd) {
        drive = rd;
    }
    
    public void initializeSecondaryDrive(int l2, int r2) {
        drive2 = new RobotDrive(l2, r2);
        hasSecondary = true;
    }
    
    public void setSecondaryDriver(RobotDrive rd) {
        drive2 = rd;
        hasSecondary = true;
    }
    
    public void arcadeDrive(Joystick joystick){
        drive.arcadeDrive(joystick);
    }
    
    public void arcadeDrive(double forward, double turn) {
        drive.arcadeDrive(forward, turn);
        if (hasSecondary) drive2.arcadeDrive(forward, turn);
    }
    
    public void tankDrive(double x, double y) {
        drive.tankDrive(x, y);
        if (hasSecondary) drive2.tankDrive(x, y);
    }
    
    public void addVictor(int port) {
        raw[port] = new Victor(port);
    }
    
    public void set(int vic, double power) {
        raw[vic].set(power);
    }
    
    public double getPwm(int vic) {
        return raw[vic].get();
    }
    
    public Victor getVictor(int vic) {
        return raw[vic];
    }
    
    public void setSafety(boolean b){
        drive.setSafetyEnabled(false);
        if (hasSecondary) drive2.setSafetyEnabled(false);
    }
}
