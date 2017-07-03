package com.nickschatz.dronecontrol;

/**
 * Each control packet to the drone is 5 bytes of data.
 * The first three are the signed command directions for the drone in 2's complement
 * Yaw/Pitch/Roll
 * The fourth is throttle
 * The last is a special command byte, usually 0
 */
public class ControlPacket {
    public final byte yawCommand;
    public final byte pitchCommand;
    public final byte rollCommand;
    public final byte throttleCommand;
    public final byte specialCommand;

    public ControlPacket(byte yawCommand, byte pitchCommand, byte rollCommand, byte throttleCommand,
                         byte specialCommand) {
        this.yawCommand = yawCommand;
        this.pitchCommand = pitchCommand;
        this.rollCommand = rollCommand;
        this.throttleCommand = throttleCommand;
        this.specialCommand = specialCommand;
    }

    public byte[] pack() {
        return new byte[] {this.yawCommand, this.pitchCommand, this.rollCommand,
                this.throttleCommand, this.specialCommand};
    }

    public String toString() {
        return "Yaw: " + this.yawCommand +
                " Pitch: " + this.pitchCommand +
                " Roll: " + this.rollCommand +
                " Throttle: " + this.throttleCommand +
                " S: " + this.specialCommand;
    }
}
