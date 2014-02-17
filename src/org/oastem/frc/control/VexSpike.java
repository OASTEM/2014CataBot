/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.control;

import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Relay;
/**
 *
 * @author STEM
 */
public class VexSpike{
    
    private Relay vexRelay;
    
    public VexSpike(int channel){
        vexRelay = new Relay(channel); //7, 8
    }
    
    public void activate(){
        vexRelay.set(Relay.Value.kOn);
    }
    
    public void goForward(){
        vexRelay.set(Relay.Value.kForward);
    }
    
    public void goBackward() {
        vexRelay.set(Relay.Value.kReverse);
    }

    
    public void free(){
        vexRelay.free();
    }
    
    public void deactivate(){
        vexRelay.set(Relay.Value.kOff);
    }
}
