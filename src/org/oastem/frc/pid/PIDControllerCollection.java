/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.pid;

import edu.wpi.first.wpilibj.PIDController;

/**
 *
 * @author KTOmega
 */
public class PIDControllerCollection {
    private PIDController[] ctrl;
    private int i = 0;
    private int max;
    
    public PIDControllerCollection(int size) {
        ctrl = new PIDController[size];
        max = size;
    }
    
    public void add(PIDController c) {
        if (i >= max) {
            throw new IndexOutOfBoundsException("Reached the limit of " + 
                    max + " PID controllers!");
        }
        ctrl[i++] = c;
    }
    
    public PIDController get(int index) {
        return ctrl[index];
    }
    
    public boolean isEnabled(int index) {
        return ctrl[index].isEnable();
    }
    
    public void enableAll() {
        for (int j = 0; j < max; j++) {
            ctrl[j].enable();
        }
    }
    
    public void disableAll() {
        for (int j = 0; j < max; j++) {
            ctrl[j].disable();
        }
    }
    
    public void freeAll() {
        for (int j = 0; j < max; j++) {
            ctrl[j].free();
        }
    }
}
