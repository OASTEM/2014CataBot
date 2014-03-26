
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
import com.sun.squawk.util.MathUtils;

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
    public static final int INTAKE_SPIKE_2 = 3;
    public static final int WINCH_SPIKE = 8;
    public static final int TRIGGER_PORT = 2;
    // Buttons:
    // Left Joystick
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
    public static final int TOGGLE_DECEL_BUTTON = 9;
    // Right Joystick
    public static final int EMERGENCY_STOP_BUTTON_RIGHT = 9;
    public static final int PULLBACK_BUTTON = 6;
    public static final int TRIGGER_BUTTON_PULL_BACK = 10;
    
    
    private static final double TRIGGER_SPEED_UP = -0.75;
    private static final double TRIGGER_SPEED_DOWN = 0.25;
    private DriveSystemAccel drive = DriveSystemAccel.getAcceleratedInstance();
    private VexSpike intake;
    private VexSpike intake2; //special spike
    private VexSpike winch;
    
    public static final long AUTO_MOVE_TIME = 700L;
    public static final long AUTO_MOVE_TIME_AGAIN = 1000L;
    // User Control States
    public static final int START = 0;
    public static final int STARTPULL = 1;
    public static final int READY = 2;
    public static final int RELEASE = 3;
    public static final int FIRING = 4;
    public static final int RESET = 5;
    public static final int PULL = 6;
    public static final int LOOSEN = 7;
    public static final int NOT_READY = 8;
    public static final int ADDITION_RESET = 9;
    public static final String[] STATE_ARRAY = {
        "Start",
        "releasing intakes",
        "ready to fire",
        "releasing winchGear",
        "firing this mofo",
        "putting things back",
        "this winch be tripping",
        "much loose",
        "We aint ready",
        "one more time"
    };
    private int state;
    // Autonomous States
    public static final int AUTO_START = 0;
    public static final int MOVE_FORWARD = 1;
    public static final int SHOOT = 2;
    public static final int AFTER_MOVE = 3;
    public static final int DONE = 4;
    public static final int PULL_DOWN = 5;
    public static final int MOVE_AGAIN = 6;
    public static final String[] AUTO_STATE_ARRAY = {
        "Start",
        "Moving Forward",
        "Shoot",
        "After Move",
        "Done",
        "Pull Down",
        "Move Again"
    };
    private int autoState;
    // corresponding distance from using pixel height of vertical vision target
    public static final int TWO_METER = 211;
    public static final int THREE_METER = 175;
    public static final int FOUR_METER = 136;
    public static final int FIVE_METER = 111;
    public static final int SHOOT_METER = 101; //lol (5.46-5.48 meters)
    public static final int SIX_METER = 92;
    public static final double PULL_DOWN_POWER = 0.50;
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
    //private DigitalInput bottomLim = new DigitalInput(3); //gablooza
    private long lastUpdate;
    private String[] debug = new String[6];
    private Joystick left = new Joystick(1);
    private Joystick right = new Joystick(2);
    private Victor trigger;
    private double joyScale = 1.0;
    private double joyScale2 = 1.0;
    private long ticks = 0;
    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    // we are storing the centers of masses
    public Point[] massCenters = null;
    private boolean deceled = false;
    private double horzCenterMassX, horzCenterMassY, vertCenterMassX, vertCenterMassY;
    private int currentHotGoal = Point.INVALID;
    private int vertHeight;
    
    private long releaseCount = 0L;

    protected void robotInit() {
        
        Debug.clear();
        Debug.log(1, 1, "Robot Initalized");
        state = START;
        lastUpdate = System.currentTimeMillis();
        motorTime = 0L;
        vertHeight = 0;

        drive.initializeDrive(LEFT_DRIVE_FRONT, LEFT_DRIVE_REAR, RIGHT_DRIVE_FRONT, RIGHT_DRIVE_REAR);
        //drive.initializeDrive(6, 4);
        drive.setSafety(false);
        intake = new VexSpike(INTAKE_SPIKE);
        intake2 = new VexSpike(INTAKE_SPIKE_2); //special spike
        intake2.deactivate(); //special spike
        intake.deactivate();
        
        winch = new VexSpike(WINCH_SPIKE);
        drive.addVictor(TRIGGER_PORT);
        drive.addVictor(WINCH_PORT);
        //trigger = new Victor(TRIGGER_PORT);

        camera = AxisCamera.getInstance("10.40.79.11");


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
        long shootingTime = 0L;
        long finalMoveTime = 0L;
        while (isAutonomous() && isEnabled()) {
            long currentTime = System.currentTimeMillis();
            imageProcessing();
            debug[2] = AUTO_STATE_ARRAY[autoState];
            switch (autoState) {
                case AUTO_START:
                    //autoMove();
                    //Determine left or right and set appropriate state
                    //if(autoPulling(currentTime, triggerStart))
                    autoState = PULL_DOWN;
                    triggerStart = currentTime;
                    finalMoveTime = currentTime;
                    break;
                case PULL_DOWN :
                    if(autoPulling(currentTime, triggerStart)){
                        autoState = MOVE_FORWARD;
                        triggerStart = currentTime;
                    }
                case MOVE_FORWARD:
                    //move forward
                    /**
                    boolean autoDone = false;
                    boolean pullDone = false;
                    if (autoMove((SHOOT_METER - 10)) || (currentTime - finalMoveTime) >= 5000L ) {
                        quickDecel();
                        autoDone = true;
                    }
                    if(autoPulling(currentTime, triggerStart)){
                        pullDone = true;
                    }
                    if(autoDone == true && pullDone == true){
                        triggerStart = currentTime;
                        shootingTime = currentTime;
                        finalMoveTime = 0L;
                        autoState = SHOOT;
                    }
                    //*/
                    
                    if(autoMoveTime(currentTime, AUTO_MOVE_TIME)){
                        autoState = SHOOT;
                        triggerStart = currentTime;
                        state = RELEASE;
                    }//*/
                    break;
                case SHOOT:
                    if(loosening(currentTime, triggerStart)){
                        if ( currentHotGoal == Point.LEFT || currentHotGoal == Point.RIGHT || (currentTime - shootingTime) >= 5000L ) {
                            autoState = AFTER_MOVE;
                            triggerStart = currentTime;
                            state = RELEASE;
                        }
                    }
                    
                    
                    break;
                case AFTER_MOVE:
                    //move again
                    //autoStates(currentTime);
                    /**if (autoMove(THREE_METER) || (currentTime - finalMoveTime) >= 5000L ) {
                        autoState = DONE;
                    }
                    //*/
                    /**
                    if(autoPulling(currentTime, triggerStart)){
                        autoState = DONE;
                    }
                    //*/
                    //triggerStart = currentTime;
                     autoStatesAuto(currentTime);
                            debug[3] = "#shotsfired";
                            //finalMoveTime = currentTime;
                   //autoState = DONE;
                    break;
                case MOVE_AGAIN :
                    if(autoMoveTime(currentTime, AUTO_MOVE_TIME_AGAIN)){
                        autoState = DONE;
                        debug[3] = "again";
                    }
                case DONE:
                    //autoStates(currentTime);
                    //u wot m8
                    debug[3] = "I'm finished";
                    debug[4] = "be gentle senpai";
                    triggerStart = 0L;
                    break;
                /**
                 * case LEFT: Debug[1] = AUTO_STATE_ARRAY[autoState]; //move
                 * until goal range if (!canShoot(vertHeight) && !hasFired){
                 * autoMove(); } //wait till hot else if (curGoal.isHot &&
                 * !hasFired){ //set shootings states state = RELEASE; //shoot
                 * autoStates(currentTime); hasFired = true;
                 * System.out.println("#Shotsfired"); } else if(hasFired &&
                 * vertHeight > THREE_METER){ //move forward and make sure it
                 * doesn't crash in some manner autoMove(); } else autoState =
                 * DONE;
                 *
                 * break; case RIGHT: //Right code break;
                 */
                default:
                    System.out.println("This code doesn't work. (autonomous() switch thing).");
                    break;
            }

            Debug.log(debug);
            //lastUpdate = System.currentTimeMillis();
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        debug[0] = "User Control Mode";
        triggerStart = 0L;
        boolean triggerHasFired = false;
        boolean intakePressed = false;
        boolean outtakePressed = false;
        boolean winchPressed = false;
        boolean winchMovePressed = false;
        //boolean winchWiggled = false;
        //boolean afterFired = false;
        //int winchWiggleCount = 0;
        long secondaryTriggerStart = 0L;
        long decelStart = 0L;
        rightDrive = 0.0;
        leftDrive = 0.0;
        currSpeedRight = 0.0;
        currSpeedLeft = 0.0;
        state = READY;
        deceled = true;
        while (isOperatorControl() && isEnabled()) {
            //imageProcessing();
            //double speed = left.getY() * joyScale;
            long currentTime = System.currentTimeMillis();
            joyScale = scaleZ(left.getZ());
            joyScale2 = scaleZ(right.getZ());
            //debug[1] = "Speed: " + speed;
            debug[1] = STATE_ARRAY[state];

            
            if (right.getRawButton(TRIGGER_BUTTON_PULL_BACK)){
                if(fireLim.get()){
                    drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
                    debug[2] = "I'm going back";
                }
                else{
                    drive.set(TRIGGER_PORT, 0.0);
                    Debug.clearLine(2);
                    debug[2] = "Stop pushing me";
                }
            }
            if (left.getRawButton(TOGGLE_DECEL_BUTTON)) {
                if (deceled && ( currentTime - decelStart >= 2000L) ) {
                    deceled = false;//drive.setDriveDecelSkip(false);
                    decelStart = currentTime;
                } 
                else if(currentTime - decelStart <= 2000L){
                    //deceled = false;
                    //nothing
                }
                else {
                    decelStart = currentTime;
                    deceled = true;
                    //drive.setDriveDecelSkip(true);
                }
            }
            /**
            if (canShoot(SHOOT_METER)) {
                debug[2] =  " SHOOT!";
            } 
            else {
                Debug.clearLine(2);
            }
            //*/

            if (left.getRawButton(WINCH_BUTTON_UP)) {
                drive.set(WINCH_PORT, 1.0);
                winchMovePressed = true;
            } else if (left.getRawButton(WINCH_BUTTON_DOWN)) {
                drive.set(WINCH_PORT, -1.0);
                winchMovePressed = true;
            }

            /**
            if (winchMovePressed && (!left.getRawButton(WINCH_BUTTON_UP)
                    || !left.getRawButton(WINCH_BUTTON_DOWN))) {
                drive.set(WINCH_PORT, 0);
                winchMovePressed = false;
            }//*/

            if (left.getRawButton(EMERGENCY_STOP_BUTTON) || right.getRawButton(EMERGENCY_STOP_BUTTON_RIGHT)) {
                // HOLY CRAP STOP
                drive.set(TRIGGER_PORT, 0);
                winch.deactivate();
                intake.deactivate();
                intake2.deactivate();
                drive.set(WINCH_PORT, 0);
                triggerHasFired = false;
                intakePressed = false;
                outtakePressed = false;
                winchPressed = false;
                winchMovePressed = false;
                //winchWiggled = false;
                //afterFired = false;
                //winchWiggleCount = 0;
                triggerStart = 0L;
                secondaryTriggerStart = 0L;
                state = NOT_READY;
                autoState = DONE;
                wiggleWinch(0);
                deceled = false;
                drive.tankDrive(0.0,0.0,deceled);
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
                intake2.goForward();
                intakePressed = true;
            } else if (!left.getRawButton(INTAKE_BUTTON) && intakePressed) {
                intake.deactivate();
                intake2.deactivate();
                intakePressed = false;
            }

            // Outtake //NO OUTTAKIG
            if (left.getRawButton(OUTTAKE_BUTTON)) {
                intake.goBackward();
                intake2.goBackward();
                outtakePressed = true;
            } else if (!left.getRawButton(OUTTAKE_BUTTON) && outtakePressed) {
                intake.deactivate();
                intake2.deactivate();
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

//println(winchLin.get());
            switch (state) {
                case READY:
                    if (left.getRawButton(TRIGGER_BUTTON)) {
                        state = RELEASE;
                        triggerStart = currentTime;
                        releaseCount = 0L;
                    }
                    if (winchLin.get()){
                        state = NOT_READY;
                        triggerStart = 0L;
                    }
                    break;
                case NOT_READY :
                    if (right.getRawButton(PULLBACK_BUTTON)){
                        state = PULL;
                        triggerStart = currentTime;
                    }
                    if (!winchLin.get()){
                        state = READY;
                        triggerStart = 0L;
                    }
                    break;
                default:
                    autoStates(currentTime);
                    break;
            }

            /**
             * TODO: Rewrite this to a switch case
             
            if (left.getRawButton(SECONDARY_TRIGGER_BUTTON) || triggerStart > 0L) {
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
            }//*/
            
            
            if (left.getRawButton(TRIGGER_BUTTON_UP)) {
                // Window motor positive
                drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
            } else if (left.getRawButton(TRIGGER_BUTTON_DOWN)) {
                // Window motor negative
                drive.set(TRIGGER_PORT, TRIGGER_SPEED_UP);
            } else if (triggerStart == 0L) {
                //trigger.set(0);
                drive.set(TRIGGER_PORT, 0);
            }//*/
//System.out.println(fireLim.get());

            long timeDelta = currentTime - lastUpdate;
            if (timeDelta > 250) {
                Debug.clear();
                ticks = currentTime;
            }

            Debug.log(debug);
            lastUpdate = System.currentTimeMillis();
        }
    }

    public double angleTurned(double oldX, double newX, double total) {
        double oneDeg = total / 80;
        return (newX - oldX) / oneDeg;
    }

    public boolean canShoot(int pixelSize) {
        if (vertHeight >= pixelSize) {
            //drive.setDriveDecelSkip(true);
            //deceled = true;
            return true;
        }
        return false;
    }

    public boolean autoMove(int pix) {
        if (canShoot(pix)) {
            quickDecel();
            return true;
        } else {
            drive.tankDrive(0.5, 0.5, deceled);
            return false;
        }
    }
    public boolean autoMoveTime(long time, long moveTime){
        if(time - triggerStart < moveTime){
            drive.tankDrive(0.5,0.5,deceled);
            return false;
        }
        else{
            quickDecel();
            return true;
        }
        
    }
    
    private void quickDecel(){
        deceled = true;
        drive.tankDrive(0, 0, deceled);
        deceled = false;
    }

    private void imageProcessing() {
        try {
            // 43:32
            //ColorImage image = camera.getImage();     // comment if using stored images
            ColorImage image;                           // next 2 lines read image from flash on cRIO
            image = camera.getImage();
            //image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
            BinaryImage thresholdImage = image.thresholdRGB(0, 50, 100, 255, 0, 50);
            //BinaryImage thresholdImage = image.thresholdHSV(60, 100, 90, 255, 20, 255);   // keep only red objects
            //thresholdImage.write("/threshold.bmp");
            BinaryImage convexHullImage = thresholdImage.convexHull(false);          // fill in occluded rectangles
            //convexHullImage.write("/convexHull.bmp");
            BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // filter out small particles
            //filteredImage.write("/filteredImage.bmp");
            //SmartDashboard.

            //iterate through each particle and score to see if it is a target
            Point scores[] = new Point[filteredImage.getNumberParticles()];
            for (int i = 0; i < scores.length; i++) {
                ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                scores[i] = new Point(i, report.center_mass_x_normalized, report.center_mass_y_normalized);

                scores[i].rectangularity = ImagingUtils.scoreRectangularity(report);
                scores[i].aspectRatioOuter = ImagingUtils.scoreAspectRatio(filteredImage, report, i, true);
                scores[i].aspectRatioInner = ImagingUtils.scoreAspectRatio(filteredImage, report, i, false);
                scores[i].xEdge = ImagingUtils.scoreXEdge(thresholdImage, report);
                scores[i].yEdge = ImagingUtils.scoreYEdge(thresholdImage, report);

                if (scores[i].aspectRatioOuter > 1.0) {
                    // Width > height, it's the horizontal goal
                    horzCenterMassX = report.center_mass_x_normalized;
                    horzCenterMassY = report.center_mass_y_normalized;
                    System.out.println(i + ": HorizGoal cx: " + report.center_mass_x_normalized + " cy: "
                            + report.center_mass_y_normalized);

                } else {
                    // Height > width, it's the vertical goal
                    vertCenterMassX = report.center_mass_x_normalized;
                    vertCenterMassY = report.center_mass_y_normalized;
                    System.out.println(i + ": VertGoal cx: " + report.center_mass_x_normalized + " cy: "
                            + report.center_mass_y_normalized
                            + " h: " + (report.boundingRectHeight / (double) report.imageHeight));
                    vertHeight = report.boundingRectHeight;
                    // This function seems to work only between 2-5 meters.
                    //double dist = 129166.84601965 * ( MathUtils.pow(report.boundingRectHeight, -1.172464652462));
                    System.out.println(report.boundingRectHeight);
                    //System.out.println( (347.5 * report.boundingRectHeight) / 92.0 );
                }



                // The following code will only store the initial readings.
                /*
                 * if (massCenters == null) { // We'll only take in the initial
                 * reading. massCenters = scores; }//
                 */

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



                /*
                 * if(scoreCompare(scores[i], false)) {
                 * System.out.println("particle: " + i + "is a High Goal
                 * centerX: " + report.center_mass_x_normalized + "centerY: " +
                 * report.center_mass_y_normalized);
                 * System.out.println("Distance: " +
                 * computeDistance(thresholdImage, report, i, false)); } else if
                 * (scoreCompare(scores[i], true)) {
                 * System.out.println("particle: " + i + "is a Middle Goal
                 * centerX: " + report.center_mass_x_normalized + "centerY: " +
                 * report.center_mass_y_normalized);
                 * System.out.println("Distance: " +
                 * computeDistance(thresholdImage, report, i, true)); } else {
                 * System.out.println("particle: " + i + "is not a goal centerX:
                 * " + report.center_mass_x_normalized + "centerY: " +
                 * report.center_mass_y_normalized); }
                 */
                //System.out.println("rect: " + scores[i].rectangularity + "ARinner: " + scores[i].aspectRatioInner);
                //System.out.println("ARouter: " + scores[i].aspectRatioOuter + "xEdge: " + scores[i].xEdge + "yEdge: " + scores[i].yEdge);	
            }

            massCenters = scores;
            ImagingUtils.determineGoals(scores);

            for (int i = 0; i < scores.length; i++) {
                Point cur = scores[i];
                String o = cur.getOrientation() == Point.INVALID ? "Invalid"
                        : (cur.getOrientation() == Point.HORIZONTAL ? "Horiz" : "Vert");
                String s = cur.getSide() == Point.INVALID ? "Invalid"
                        : (cur.getSide() == Point.LEFT ? "Left" : "Right");

                String h = cur.isHot() ? "Hot" : "NotHot";

                if (cur.isHot()) {
                    currentHotGoal = cur.getSide();
                }

                System.out.println("Goal " + i + ": " + o + " " + s + " " + h);
            }

            //public void checkRegion(
            //System.out.println(ImagingUtils.isRightOrLeft(vertCenterMassX, vertCenterMassY, horzCenterMassX, horzCenterMassY) + "");

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
                    state = ADDITION_RESET;
                }
                break;
            case ADDITION_RESET :
                if (fireLimReset(currTime, triggerStart)){
                    triggerStart = 0L;
                    state = NOT_READY;
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
                    triggerStart = 0L;
                    state = READY;
                    autoState = AFTER_MOVE;
                }
                break;
            default:
                break; //nothing should be happening here
            }
    }
    
    private void autoStatesAuto(long currTime) {
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
            case ADDITION_RESET :
                if (fireLimReset(currTime, triggerStart)){
                    triggerStart = 0L;
                    state = NOT_READY;
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
                    triggerStart = currTime;
                    state = READY;
                    autoState = MOVE_AGAIN;
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
        System.out.println(currTime +":"+triggerStart);
        if (currTime - trigStart > 500L) {
            wiggleWinch(0);
            System.out.println(releaseCount);
            releaseCount = 0L;
            return true;
        } else {
            releaseCount = releaseCount + 1;
            wiggleWinch(-0.10);
        }
        return false;
    }

    private boolean firing(long currTime, long trigStart) {
        if (currTime - trigStart > 1800L) {
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
            return true;
        }
        return false;
    }
    
    private boolean fireLimReset(long currTime, long trigStart){
        if (fireLim.get() && currTime - trigStart < 5200L) {
                //System.out.println("I'm here and resseting");//reference
           drive.set(TRIGGER_PORT, TRIGGER_SPEED_DOWN);
           return false;
       }
       drive.set(TRIGGER_PORT, 0);
       return true;
    }
//gotta change all this stuff gablooza
    private boolean pulling(long currTime, long trigStart) {
        if (currTime - trigStart < 6000L && winchLin.get()) {
            wiggleWinch(PULL_DOWN_POWER);
            return false;
        }
        wiggleWinch(0);
        return true;
    }
    
    private boolean autoPulling(long currTime, long trigStart){
        if (currTime - trigStart < 600L) { // bottomLim.get()
            wiggleWinch(PULL_DOWN_POWER);
            return false;
        }
        wiggleWinch(0);
        return true;
    }
    
   

    private boolean loosening(long currTime, long trigStart) {
        if (currTime - trigStart < 500L){// && !bottomLim.get()) {
            wiggleWinch(-0.2);
            return false;
        }//
        wiggleWinch(0);
        return true;
    }

    private void doingWinchStuff(String[] debug) {
        if(state == READY || state == NOT_READY){
        double y = right.getY();
        double winchMove = 0.0;
        double zone = 0.04;
        if (Math.abs(y) > zone) {
            winchMove = y;
        }
        winchMove *= joyScale2 * -1;    

        //debug[5] = "rScale: " + joyScale2 + " Winch: " + winchMove;
        
        drive.set(WINCH_PORT, winchMove);
        }
    }

    private void wiggleWinch(double pow) {
        drive.set(WINCH_PORT, pow);
    }

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
        debug[4] = "Left: " + getDriveSpeed(LEFT_DRIVE_FRONT);
        debug[5] = "Right: " + getDriveSpeed(RIGHT_DRIVE_FRONT);
        //debug[4] = "Left: " + getDriveSpeed(6) + " Right: " + getDriveSpeed(4); //change this
        drive.tankDrive(leftMove, rightMove, deceled);
    }

    /**
    private void autoDrive(double a, double b, String[] debug) {
        debug[4] = "Left: " + getDriveSpeed(LEFT_DRIVE_FRONT) + " Right: " + getDriveSpeed(RIGHT_DRIVE_FRONT);
        drive.tankDrive(a, b, deceled);
    }
    * //*/

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
