package org.vulcanrobotics.robotcorelib.subsystems;

public enum PowerShot {
    LEFT(0),
    RIGHT(2),
    CENTER(1),
    NONE(-1);

    public int id;

    PowerShot(int id) {
        this.id = id;
    }
}
