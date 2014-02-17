/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.pid;

import org.oastem.frc.pid.PIDGainOutput;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import org.oastem.frc.assist.RobotMain;
import org.oastem.frc.control.DriveSystem;

/**
 *
 * @author KTOmega
 */
public class TargetOutput {
    private DriveSystem drive;
    
    // Connect this to angle output
    public PIDGainOutput angleRobotToGoal = new PIDGainOutput() {
        private double zone = 0.05;
        
        public void pidWrite(double output) {
            // angle is negative when goal is to the left.
            // angle is positive when goal is to the right.
            System.out.println("Angler PID Output: " + output);
            if (output < -zone) {
                drive.tankDrive(-output, output);
            } else if (output > zone) {
                drive.tankDrive(output, -output);
            } else {
                drive.tankDrive(0, 0);
            }
        }

        public double getKp() {
            return 0.5;
        }

        public double getKi() {
            return 0.2;
        }

        public double getKd() {
            return 0.1;
        }
    };
    
    // Connect this to width output
    public PIDGainOutput driveRobotToGoal = new PIDGainOutput() {
        private double zone = 0.5;
        private double forward = 1.0;
        private double speed = 0.33;
        
        public void pidWrite(double output) {
            double goalDist = 100;
            System.out.println("Driver PID Output: " + output);
            if (output > goalDist) {
                // too close, back up
                drive.tankDrive(-speed, speed);
            } else if (output < goalDist) {
                // too far, go forward
                drive.tankDrive(speed, speed);
            } else {
                // yeeee
                drive.tankDrive(0, 0);
            }
        }

        public double getKp() {
            return 0.5;
        }

        public double getKi() {
            return 0.2;
        }

        public double getKd() {
            return 0.1;
        }
    };
    
    // Connect this to height output
    public PIDGainOutput angleTraamToGoal = new PIDGainOutput() {
        private double zone = 0.05;
        private double targetHeight = 120; // in pixels
        
        public void pidWrite(double output) {
            double delta = output - targetHeight;
            double outPower = 0.0;
            if (delta > zone) {
                // goal above target
                outPower = -1.0;
            } else if (delta < -zone) {
                // goal below target
                outPower = 1.0;
            } else {
                // on target
                outPower = 0.0;
            }
            
            //drive.set(RobotMain.TRAAM, outPower);
        }

        public double getKp() {
            return 0.0;
        }

        public double getKi() {
            return 0.0;
        }

        public double getKd() {
            return 0.0;
        }
    };
    
    // Connect this to height output
    public PIDGainOutput powerWheelForGoal = new PIDGainOutput() {
        private double zone = 0.05;
        private double targetHeight = 120; // in pixels
        
        public void pidWrite(double output) {
            double delta = output - targetHeight;
            double outPower = 0.0;
            if (delta > zone) {
                // goal above target
                outPower = Math.min(Math.abs(delta) * 2, 1.0);
            } else {
                // on target
                outPower = 0.0;
            }
            
            //drive.set(RobotMain.SHOOTER_WHEEL, outPower);
        }

        public double getKp() {
            return 0.0;
        }

        public double getKi() {
            return 0.0;
        }

        public double getKd() {
            return 0.0;
        }
    };
    
    public TargetOutput() {
        drive = DriveSystem.getInstance();
    }
}
