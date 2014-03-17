/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.oastem.frc.assist;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.oastem.frc.Debug;
import org.oastem.frc.control.*;
import org.oastem.frc.imaging.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends SimpleRobot {
    // Ports for Victor
    public static final int WINCH_PORT = 1;
    public static final int RIGHT_DRIVE_FRONT = 4;
    public static final int RIGHT_DRIVE_REAR = 5;
    public static final int LEFT_DRIVE_REAR = 7;
    public static final int LEFT_DRIVE_FRONT = 6;
    public static final int INTAKE_SPIKE = 7;
    public static final int WINCH_SPIKE = 8;
    public static final int TRIGGER_PORT = 2;
    
    // Buttons:
    public static final int INTAKE_BUTTON = 6;
    public static final int TRIGGER_BUTTON = 1;
    public static final int OUTTAKE_BUTTON = 7;
    public static final int TRIGGER_BUTTON_UP = 3;
    public static final int TRIGGER_BUTTON_DOWN = 2;
    public static final int WINCH_BUTTON_UP = 11;
    public static final int WINCH_BUTTON_DOWN = 10;
    public static final int WINCH_BUTTON_SPIKE = 8;
    public static final int EMERGENCY_STOP_BUTTON = 4;
    public static final int SECONDARY_FIRE_BUTTON = 5;
    private static final double TRIGGER_SPEED_UP = -0.75;
    private static final double TRIGGER_SPEED_DOWN = 0.25;
    private DriveSystemAccel drive = DriveSystemAccel.getAcceleratedInstance();
    private VexSpike intake;
    private VexSpike winch;
    
    // User Control States
    public static final int START = 0;
    public static final int STARTPULL = 1;
    public static final int READY = 2;
    public static final int RELEASE = 3;
    public static final int FIRING = 4;
    public static final int RESET = 5;
    public static final int PULL = 6;
    public static final int LOOSEN = 7;
    public static final String[] STATE_ARRAY = {
        "Start",
        "releasing intakes",
        "ready to fire",
        "releasing winchGear",
        "firing this mofo",
        "putting things back",
        "this winch be tripping",
        "much loose"
    };
    private int state;
    
    // Autonomous States
    public static final int AUTO_START = 0;
    public static final int LEFT = 1;
    public static final int RIGHT = 2;
    public static final int CENTER = 3;
    public static final String[] AUTO_STATE_ARRAY = {
        "Start",
        "Left",
        "Right",
        "Center"
    };
    
    private int autoState;
    private long accelTime;
    // acceleration per millisecond
    private long triggerStart = 0L;
    public static final double ACCEL_FACTOR = 0.00033;
    // Vf = Vo + at
    private double leftDrive;
    private double rightDrive;
    private double currSpeedLeft;
    private double currSpeedRight;
    private long motorTime;
    private double THRESHOLD = 0.0005;
    private DigitalInput fireLim = new DigitalInput(1);
    private DigitalInput winchLin = new DigitalInput(2);
    private long lastUpdate;
    private String[] debug = new String[6];
    private Joystick left = new Joystick(1);
    private Joystick right = new Joystick(2);
    //private Victor trigger;
    private double joyScale = 1.0;
    private double joyScale2 = 1.0;
    private long ticks = 0;

    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    // we are storing the centers of masses
    public double[][] massCenters = new double[4][2];
    private double horzCenterMassX, horzCenterMassY, vertCenterMassX, vertCenterMassY;

    protected void robotInit() {
        Debug.clear();
        Debug.log(1, 1, "Robot Initalized");
        state = START;
        lastUpdate = System.currentTimeMillis();
        motorTime = 0L;

        drive.initializeDrive(LEFT_DRIVE_FRONT, LEFT_DRIVE_REAR, RIGHT_DRIVE_FRONT, RIGHT_DRIVE_REAR);
        drive.setSafety(false);
        intake = new VexSpike(INTAKE_SPIKE);
        winch = new VexSpike(WINCH_SPIKE);
        drive.addVictor(TRIGGER_PORT);
        drive.addVictor(WINCH_PORT);
        //trigger = new Victor(TRIGGER_VICTOR);

        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, 400, 65535, false);
        //camera = AxisCamera.getInstance("10.40.79.11");
        horzCenterMassX = 0.0;
        horzCenterMassY = 0.0;
        vertCenterMassX = 0.0;
        vertCenterMassY = 0.0;

        System.out.println("End of Robot Init");
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        debug[0] = "Autonomous mode";
        autoState = AUTO_START;
        while (isAutonomous() && isEnabled()) {
            switch (autoState) {
                case AUTO_START:
                    imageProcessing();
                    lastUpdate = System.currentTimeMillis();
            }
            Debug.log(debug);
        }
    }

    /**
     * This function is called once each time the robot enters operator
     * control.
     */
    public void operatorControl() {
        debug[0] = "User Control Mode";
        boolean triggerHasFired = false;
        boolean intakePressed = false;
        boolean outtakePressed = false;
        boolean winchPressed = false;
        boolean winchMovePressed = false;
        boolean winchWiggled = false;
        boolean afterFired = false;
        int winchWiggleCount = 0;
        long secondaryTriggerStart = 0L;
        rightDrive = 0.0;
        leftDrive = 0.0;
        currSpeedRight = 0.0;
        currSpeedLeft = 0.0;
        while (isOperatorControl() && isEnabled()) {
            double speed = left.getY() * joyScale;
            long currentTime = System.currentTimeMillis();
            joyScale = scaleZ(left.getZ());
            joyScale2 = scaleZ(right.getZ());
            debug[1] = "Speed: " + speed;
            debug[2] = STATE_ARRAY[state];

            if (left.getRawButton(WINCH_BUTTON_UP)) {
                drive.set(WINCH_PORT, 1.0);
                winchMovePressed = true;
            } else if (left.getRawButton(WINCH_BUTTON_DOWN)) {
                drive.set(WINCH_PORT, -1.0);
                winchMovePressed = true;
            }

            if (winchMovePressed && (!left.getRawButton(WINCH_BUTTON_UP)
                    || !left.getRawButton(WINCH_BUTTON_DOWN))) {
                drive.set(WINCH_PORT, 0);
                winchMovePressed = false;
            }

            if (left.getRawButton(EMERGENCY_STOP_BUTTON)) {
                // HOLY CRAP STOP
                drive.set(TRIGGER_PORT, 0);
                drive.set(WINCH_PORT, 0);
                winch.deactivate();
                winchMovePressed = false;
                triggerHasFired = false;
                triggerStart = 0L;
                debug[0] = "Soft Emergency Stop";
                Debug.log(debug);
                return;
            }

            if (right.getRawButton(9)) {
                winch.deactivate();
                drive.set(WINCH_PORT, 0);
                triggerHasFired = false;
                intakePressed = false;
                outtakePressed = false;
                winchPressed = false;
                winchMovePressed = false;
                winchWiggled = false;
                afterFired = false;
                winchWiggleCount = 0;
                triggerStart = 0L;
                secondaryTriggerStart = 0L;
            }

            if (left.getRawButton(WINCH_BUTTON_SPIKE)) {
                winch.goForward(); // hopefully yes
                winchPressed = true;
            } else if (!(left.getRawButton(WINCH_BUTTON_SPIKE)) && winchPressed) {
                winch.deactivate();
                winchPressed = false;
            }

            this.doingWinchStuff(debug);

            this.doArcadeDrive(debug);

            // Intake
            if (left.getRawButton(INTAKE_BUTTON)) {
                intake.goForward();
                intakePressed = true;
            } else if (!(left.getRawButton(INTAKE_BUTTON)) && intakePressed) {
                intake.deactivate();
                intakePressed = false;
            }

            // Outtake
            if (left.getRawButton(OUTTAKE_BUTTON)) {
                intake.goBackward();
                outtakePressed = true;
            }

            if (!(left.getRawButton(OUTTAKE_BUTTON)) && outtakePressed) {
                intake.deactivate();
                outtakePressed = false;
            }

            if (left.getRawButton(SECONDARY_FIRE_BUTTON) || secondaryTriggerStart > 0L) {
                if (secondaryTriggerStart == 0L) {
                    secondaryTriggerStart = currentTime;
                }
                if (!triggerHasFired) {
                    drive.set(TRIGGER_PORT, TRIGGER_SPEED_UP);
                }
                if (currentTime - secondaryTriggerStart > 600L && !triggerHasFired) {
                    drive.set(TRIGGER_PORT, 0);
                    triggerHasFired = true;
                    secondaryTriggerStart = currentTime;
                }
                if (triggerHasFired && currentTime - secondaryTriggerStart > 1000L) {
                    drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
                    
                    if (fireLim.get() && currentTime - secondaryTriggerStart <= 4000L) {
                        // Limit switch not engaged, but all is well
                    } else {
                        if (fireLim.get()) {
                            // Limit switch not engaged, but time exceeded...
                            debug[2] = "WTFBBQ";
                        }
                        
                        drive.set(TRIGGER_PORT, 0);
                        triggerHasFired = false;
                        secondaryTriggerStart = 0L;
                    }
                }
            }


            switch (state) {
                case READY:
                    if (left.getRawButton(TRIGGER_BUTTON)) {
                        state = RELEASE;
                        triggerStart = currentTime;
                    }
                    break;
                default:
                    autoStates(currentTime);
                    break;
            }
            /**
             * TODO: Rewrite this to a switch case
             */
            if (left.getRawButton(TRIGGER_BUTTON) || triggerStart > 0L) {
                if (triggerStart == 0L) {
                    triggerStart = currentTime;
                    winch.goForward();
                } else if (currentTime - triggerStart > 100L && !winchWiggled && !triggerHasFired && !afterFired) {
                    if (winchWiggleCount < 1) {
                        winchWiggleCount++;
                        wiggleWinch(0.10);
                    } else if (currentTime - triggerStart > 500L) {
                        winchWiggled = true;
                        wiggleWinch(0);
                    } else {
                        drive.set(TRIGGER_PORT, TRIGGER_SPEED_UP);
                    }
                } else {
                    if (afterFired && !winchWiggled) {
                        if (winchWiggleCount < 1) {
                            winchWiggleCount++;
                            wiggleWinch(0.15);
                            triggerStart = currentTime;
                        } else if (currentTime - triggerStart > 500L) {
                            wiggleWinch(0);
                            afterFired = false;
                            winchWiggleCount = 0;
                            triggerStart = 0L;
                        }
                    } else {
                        if (currentTime - triggerStart > 600L && !triggerHasFired) {
                            drive.set(TRIGGER_PORT, 0);
                            triggerHasFired = true;
                            triggerStart = currentTime;
                        }
                        if (triggerHasFired && currentTime - triggerStart > 1000L) {
                            drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
                            winch.deactivate();
                            
                            if (fireLim.get() && currentTime - triggerStart <= 4000L) {
                                // Limit switch not engaged, but all is well
                            } else {
                                if (fireLim.get()) {
                                    // Limit switch not engaged, but time exceeded...
                                    debug[2] = "WTFBBQ";
                                }

                                drive.set(TRIGGER_PORT, 0);
                                triggerHasFired = false;
                                winchWiggled = false;
                                afterFired = true;
                                winchWiggleCount = 0;
                            }
                        }
                    }
                }
            }
            
            if (left.getRawButton(TRIGGER_BUTTON_UP)) {
                // Window motor positive
                drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
            } else if (left.getRawButton(TRIGGER_BUTTON_DOWN)) {
                // Window motor negative
                drive.set(TRIGGER_PORT, TRIGGER_SPEED_UP);
            } else if (triggerStart == 0L) {
                //trigger.set(0);
                drive.set(TRIGGER_PORT, 0);
            }


            long timeDelta = currentTime - lastUpdate;
            if (timeDelta > 250) {
                Debug.clear();
                ticks = currentTime;
            }

            Debug.log(debug);
            lastUpdate = System.currentTimeMillis();
        }
    }

    private void imageProcessing() {
        try {
            // 43:32
            //ColorImage image = camera.getImage();     // comment if using stored images
            ColorImage image;                           // next 2 lines read image from flash on cRIO
            //image = camera.getImage();
            image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
            BinaryImage thresholdImage = image.thresholdRGB(0, 50, 150, 255, 100, 200);
            //BinaryImage thresholdImage = image.thresholdHSV(60, 100, 90, 255, 20, 255);   // keep only red objects
            //thresholdImage.write("/threshold.bmp");
            BinaryImage convexHullImage = thresholdImage.convexHull(false);          // fill in occluded rectangles
            //convexHullImage.write("/convexHull.bmp");
            BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // filter out small particles
            //filteredImage.write("/filteredImage.bmp");
            //SmartDashboard.

            //iterate through each particle and score to see if it is a target
            Scores scores[] = new Scores[filteredImage.getNumberParticles()];
            for (int i = 0; i < scores.length; i++) {
                ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                scores[i] = new Scores();

                scores[i].rectangularity = ImagingUtils.scoreRectangularity(report);
                scores[i].aspectRatioOuter = ImagingUtils.scoreAspectRatio(filteredImage, report, i, true);
                scores[i].aspectRatioInner = ImagingUtils.scoreAspectRatio(filteredImage, report, i, false);
                scores[i].xEdge = ImagingUtils.scoreXEdge(thresholdImage, report);
                scores[i].yEdge = ImagingUtils.scoreYEdge(thresholdImage, report);


                if (scores[i].aspectRatioOuter > 1.0) {
                    // Width > height, it's the horizontal goal
                    horzCenterMassX = report.center_mass_x_normalized;
                    horzCenterMassY = report.center_mass_y_normalized;
                    massCenters[i][0] = horzCenterMassX;
                    massCenters[i][1] = horzCenterMassY;
                    System.out.println(i + ": HorizGoal cx: " + report.center_mass_x_normalized + " cy: "
                            + report.center_mass_y_normalized);

                } else {
                    // Height > width, it's the vertical goal
                    vertCenterMassX = report.center_mass_x_normalized;
                    vertCenterMassY = report.center_mass_y_normalized;
                    massCenters[i][0] = horzCenterMassX;
                    massCenters[i][1] = horzCenterMassY;
                    System.out.println(i + ": VertGoal cx: " + report.center_mass_x_normalized + " cy: "
                            + report.center_mass_y_normalized
                            + " h: " + (report.boundingRectHeight / (double) report.imageHeight));
                    System.out.println(report.boundingRectHeight);
                    //System.out.println( (347.5 * report.boundingRectHeight) / 92.0 );
                }

                // in discovering distance. ...
                // y = distance to target (to find)
                // x = sample distance (i.e. 10 meters)
                // h = sample height of target (corresponding to sample distance)
                // z = height of target
                // y = hz / x

                // h = 92 px
                // x = 347.5 cm
                // ----------------------------------
                // DISTANCE til full view of vision targets: 78.9 inches == 200 cm!!! 2 m
                // x = FOV/140 //credits to Spring
                // x = distance from wall to robot
                // FOV = bounding rect width --> width from edge of horzgoal to other edge 
                // TEST THIS
                // Put robot 2 meters from the vision targets and measure the pixel width of the image
                // (or boundingRectWidth) (from the edges of the goals



                /*if(scoreCompare(scores[i], false))
                 {
                 System.out.println("particle: " + i + "is a High Goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
                 System.out.println("Distance: " + computeDistance(thresholdImage, report, i, false));
                 } else if (scoreCompare(scores[i], true)) {
                 System.out.println("particle: " + i + "is a Middle Goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
                 System.out.println("Distance: " + computeDistance(thresholdImage, report, i, true));
                 } else {
                 System.out.println("particle: " + i + "is not a goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
                 }*/
                //System.out.println("rect: " + scores[i].rectangularity + "ARinner: " + scores[i].aspectRatioInner);
                //System.out.println("ARouter: " + scores[i].aspectRatioOuter + "xEdge: " + scores[i].xEdge + "yEdge: " + scores[i].yEdge);	
            }

            //public void checkRegion(
            System.out.println(ImagingUtils.isRightOrLeft(vertCenterMassX, vertCenterMassY, horzCenterMassX, horzCenterMassY) + "");

            /**
             * all images in Java must be freed after they are used since they
             * are allocated out of C data structures. Not calling free() will
             * cause the memory to accumulate over each pass of this loop.
             */
            filteredImage.free();
            convexHullImage.free();
            thresholdImage.free();
            image.free();
            System.out.println("-------");
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        lastUpdate = System.currentTimeMillis();
    }


    private void autoStates(long currTime) {
        switch (state) {
            case RELEASE:
                if (release(currTime, triggerStart)) {
                    triggerStart = currTime;
                    state = FIRING;
                }
                //triggerStart = currTime;
                break;//
            case FIRING:
                if (firing(currTime, triggerStart)) {
                    triggerStart = currTime;
                    state = RESET;
                }
                break;
            case RESET:
                if (resetting(currTime, triggerStart)) {
                    triggerStart = currTime;
                    state = PULL;
                }
                break;
            case PULL:
                if (pulling(currTime, triggerStart)) {
                    triggerStart = currTime;
                    state = LOOSEN;
                }
                break;
            case LOOSEN:

                if (loosening(currTime, triggerStart)) {
                    triggerStart = 0;
                    state = READY;
                }
                break;
            default:
                break; //nothing should be happening here
            }
    }

    private double scaleZ(double rawZ) {
        return Math.min(1.0, 0.5 - 0.5 * rawZ);
    }

    private boolean release(long currTime, long trigStart) {
        winch.goForward();
        return releaseGear(currTime, trigStart);
    }

    private boolean releaseGear(long currTime, long trigStart) {
        if (currTime - trigStart > 500L) {
            wiggleWinch(0);
            return true;
        } else if (currTime - trigStart > 100L) {
            wiggleWinch(0.10);
        }
        return false;
    }

    private boolean firing(long currTime, long trigStart) {
        if (currTime - trigStart > 1600L) {
            return true;
        } else if (currTime - trigStart > 600L) {
            drive.set(TRIGGER_PORT, 0);
        } else {
            drive.set(TRIGGER_PORT, TRIGGER_SPEED_UP);
        }
        return false;
    }

    private boolean resetting(long currTime, long trigStart) {
        winch.deactivate();
        if (releaseGear(currTime, trigStart)) {
            if (!fireLim.get() && currTime - trigStart < 4000L) {
                drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
                return false;
            }
            drive.set(TRIGGER_PORT, 0);
            return true;
        }
        return false;
    }

    private boolean pulling(long currTime, long trigStart) {
        if (currTime - trigStart < 4000L && winchLin.get()) {
            wiggleWinch(0.5);
            return false;
        }
        wiggleWinch(0);
        return true;
    }

    private boolean loosening(long currTime, long trigStart) {
        if (currTime - trigStart < 400L) {
            wiggleWinch(-0.1);
            return false;
        }//
        wiggleWinch(0);
        return true;
    }

    private void doingWinchStuff(String[] debug) {
        double y = right.getY();
        double winchMove = 0.0;
        double zone = 0.04;
        if (Math.abs(y) > zone) {
            winchMove = y;
        }
        winchMove *= joyScale2 * -1;

        debug[5] = "rScale: " + joyScale2 + " Winch: " + winchMove;
        drive.set(WINCH_PORT, winchMove);
    }

    private void wiggleWinch(double pow) {
        drive.set(WINCH_PORT, pow);
    }

    /**
     * Guys we need to test this shit we would put this into the firing method
     * (at the end)
     */
    private void pullWinchBack() {
        if (!winchLin.get()) {
            drive.set(WINCH_PORT, -0.5);
        } else {
            drive.set(WINCH_PORT, 0);
        }
    }

    /**
     * private void sleepBro(int time){ try{ Thread.sleep(time); }
     * catch(Exception e){ // we should be going here }
    }//
     */
    private void doArcadeDrive(String[] debug) {
        double leftMove = 0.0;
        double rightMove = 0.0;
        double zone = 0.04;

        joyScale = scaleZ(left.getZ());

        double x = left.getX() * -1;
        double y = left.getY();

        if (Math.abs(y) > zone) {
            leftMove = y;
            rightMove = y;
        }

        if (Math.abs(x) > zone) {
            leftMove = correct(leftMove + x);
            rightMove = correct(rightMove - x);
        }

        leftMove *= joyScale * -1;
        rightMove *= joyScale * -1;
        //leftDrive = leftMove;
        //rightDrive = rightMove;
        //currSpeedLeft = accelerate(currSpeedLeft,leftDrive, timeDiff);
        //currSpeedRight = accelerate(currSpeedRight,rightDrive, timeDiff); //
        debug[3] = "Scale: " + joyScale;
        debug[4] = "Left: " + getDriveSpeed(LEFT_DRIVE_FRONT) + " Right: " + getDriveSpeed(RIGHT_DRIVE_FRONT);
        drive.tankDrive(leftMove, rightMove);
    }

    private void autoDrive(double a, double b, String[] debug) {
        debug[4] = "Left: " + getDriveSpeed(LEFT_DRIVE_FRONT) + " Right: " + getDriveSpeed(RIGHT_DRIVE_FRONT);
        drive.tankDrive(a, b);
    }

    private double getDriveSpeed(int port) {
        return drive.getAccelSpeed(port);
    }

    private double correct(double val) {
        if (val > 1.0) {
            return 1.0;
        }
        if (val < -1.0) {
            return -1.0;
        }
        return val;
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    }
}
