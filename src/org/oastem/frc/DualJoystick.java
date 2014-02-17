/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.parsing.IInputOutput;

/**
 *
 * @author KTOmega
 */
public class DualJoystick extends GenericHID implements IInputOutput {
    
    private Joystick left;
    private Joystick right;
    private Hand pref;

    public DualJoystick(int left, int right, Hand pref) {
        this.left = new Joystick(left);
        this.right = new Joystick(right);
        this.pref = pref;
    }
    
    public DualJoystick(Joystick left, Joystick right, Hand pref) {
        this.left = left;
        this.right = right;
        this.pref = pref;
    }
    
    public void setPreference(Hand pref) {
        this.pref = pref;
    }
    
    public Hand getPreference() {
        return pref;
    }

    private double nonZero(double leftVal, double rightVal) {
        if (leftVal == 0) {
            return rightVal;
        } else if (rightVal == 0) {
            return leftVal;
        } else {
            if (getPreference() == Hand.kLeft) {
                return leftVal;
            } else if (getPreference() == Hand.kRight) {
                return rightVal;
            } else {
                return Math.max(leftVal, rightVal);
            }
        }
    }

    public boolean getRawButton(int button) {
        return left.getRawButton(button) || right.getRawButton(button);
    }

    public boolean getBumper(Hand hand) {
        return false;
    }

    public boolean getTop(Hand hand) {
        return left.getTop() || right.getTop();
    }

    public double getRawAxis(int axis) {
        double leftAxis = left.getRawAxis(axis);
        double rightAxis = right.getRawAxis(axis);
        return nonZero(leftAxis, rightAxis);
    }

    public boolean getTrigger(Hand hand) {
        return left.getTrigger() || right.getTrigger();
    }

    public double getThrottle() {
        double leftVal = left.getThrottle();
        double rightVal = right.getThrottle();
        return nonZero(leftVal, rightVal);
    }

    public double getTwist() {
        double leftVal = left.getTwist();
        double rightVal = right.getTwist();
        return nonZero(leftVal, rightVal);
    }

    public double getX(Hand hand) {
        double leftVal = left.getX();
        double rightVal = right.getX();
        return nonZero(leftVal, rightVal);
    }

    public double getY(Hand hand) {
        double leftVal = left.getY();
        double rightVal = right.getY();
        return nonZero(leftVal, rightVal);
    }

    public double getZ(Hand hand) {
        double leftVal = left.getZ();
        double rightVal = right.getZ();
        return nonZero(leftVal, rightVal);
    }
}