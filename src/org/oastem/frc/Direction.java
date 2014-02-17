/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.oastem.frc;

/**
 *
 * @author STEM
 */
public class Direction {
    public static final Direction UP = new Direction(1);
    public static final Direction DOWN = new Direction(2);
    public static final Direction LEFT = new Direction(3);
    public static final Direction RIGHT = new Direction(4);
    public static final Direction NORTH = new Direction(5);
    public static final Direction EAST = new Direction(6);
    public static final Direction SOUTH = new Direction(7);
    public static final Direction WEST = new Direction(8);
    
    private int id;
    
    private Direction(int id) {
        this.id = id;
    }
    
    public boolean equals(Direction dir) {
        return this.getId() == dir.getId();
    }
    
    public int getId() {
        return id;
    }
}