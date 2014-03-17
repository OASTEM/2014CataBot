/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc.imaging;

import com.sun.squawk.util.MathUtils;

/**
 * This class is gonna contain ALL of the possible data we can get
 * on each data point we get. 
 * 
 * Tons of variables. You have been warned.
 * 
 * @author KTOmega
 */
public class Point {
    public double rectangularity;
    public double aspectRatioInner;
    public double aspectRatioOuter;
    public double xEdge;
    public double yEdge;
    public double x;
    public double y;
    public int label = 0;
    public int marked = -1;
    
    // Oh dear, oh my.
    // The things I do to make code look nice.
    public static int LEFT = 0;
    public static int RIGHT = 1;
    public static int HORIZONTAL = 0;
    public static int VERTICAL = 1;
    
    // 0 => left, 1 => right
    private int side = LEFT;
    
    // 0 => horizontal, 1 => vertical
    private int orientation = HORIZONTAL;
    
    public Point(int label, double x, double y) {
        this(x, y);
        this.setLabel(label);
    }
    
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public double getX() { return this.x; }
    public double getY() { return this.y; }
    
    public boolean isLeft() {
        return side == 0;
    }
    
    public boolean isRight() {
        return side == 1;
    }
    
    public double getAspectRatio() {
        // ???? WHICH VARIABLE IS THE ACTUAL ASPECT RATIO?
        return aspectRatioOuter;
    }
    
    public boolean hasAspect(double aspect, double error) {
        return Math.abs(this.getAspectRatio() - aspect) <= error;
    }
    
    public void setOrientation(int newOr) {
        this.orientation = newOr;
    }
    
    public int getLabel() { return this.label; }
    public void setLabel(int newLabel) { this.label = newLabel; }
    
    public boolean isMarked() { return this.marked != -1; }
    public int getMarker() { return this.marked; }
    public void setMarker(int mark) { this.marked = mark; }
    
    public static double calculateDistance(Point p1, Point p2) {
        return Math.sqrt(MathUtils.pow(p2.getX() - p1.getX(), 2) + MathUtils.pow(p2.getY() - p1.getY(), 2));
    }
    
    public String toString() {
        return "(" + this.getX() + ", " + this.getY() + ")";
    }
}
