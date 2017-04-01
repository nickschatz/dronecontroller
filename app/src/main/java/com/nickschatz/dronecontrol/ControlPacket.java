package com.nickschatz.dronecontrol;

/**
 * Each control packet to the drone is 4 bytes of data.
 * The first three are the signed command directions for the drone in 2's complement
 * The last is a special command byte, usually 0
 */
public class ControlPacket {
    public final byte xCommand;
    public final byte yCommand;
    public final byte zCommand;
    public final byte specialCommand;

    public ControlPacket(byte xCommand, byte yCommand, byte zCommand, byte specialCommand) {
        this.xCommand = xCommand;
        this.yCommand = yCommand;
        this.zCommand = zCommand;
        this.specialCommand = specialCommand;
    }

    public byte[] pack() {
        return new byte[] {this.xCommand, this.yCommand, this.zCommand, this.specialCommand};
    }

    public String toString() {
        return "X: " + Integer.toBinaryString(this.xCommand & 0xFF) +
                " Y: " + Integer.toBinaryString(this.yCommand & 0xFF) +
                " Z: " + Integer.toBinaryString(this.zCommand & 0xFF) +
                " S: " + Integer.toBinaryString(this.specialCommand & 0xFF);
    }
}
