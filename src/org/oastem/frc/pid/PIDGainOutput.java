/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.pid;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 *
 * @author KTOmega
 */
public interface PIDGainOutput extends PIDOutput {
    public double getKp();
    public double getKi();
    public double getKd();
}
