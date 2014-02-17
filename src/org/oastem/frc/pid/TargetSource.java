/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.pid;

import edu.wpi.first.wpilibj.PIDSource;

/**
 *
 * @author KTOmega
 */
public class TargetSource {
    private double[] vals;
    
    public TargetSource(double[] params) {
        vals = new double[params.length];
        update(params);
    }

    public TargetSource(int i) {
        vals = new double[i];
    }
    
    public void update(double[] params) {
        if (params.length != vals.length) {
            throw new ArrayIndexOutOfBoundsException("Parameter array length does not match internal array length");
        }
        vals = params;
    }
    
    public double get(int index) {
        return vals[index];
    }
    
    public PIDSource getSource(final int index) {
        return new PIDSource() {
            public double pidGet() {
                return vals[index];
            }
        };
    }
}
