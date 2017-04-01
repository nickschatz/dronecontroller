package com.nickschatz.dronecontrol;

import android.view.MotionEvent;

/**
 * Created by nick on 3/31/17.
 */

public class XBoxAdapter {
    private final MotionEvent ev;

    public XBoxAdapter(MotionEvent ev) {
        this.ev = ev;
    }
    public byte getLX() {
        return (byte) (ev.getX() * 127);
    }
    public byte getLY() {
        return (byte) (ev.getY() * 127);
    }
    public byte getRX() {
        return (byte) (ev.getAxisValue(MotionEvent.AXIS_Z) * 127);
    }
    public byte getRY() {
        return (byte) (ev.getAxisValue(MotionEvent.AXIS_RZ) * 127);
    }
}
