package com.nickschatz.dronecontrol;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.widget.TextView;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MainActivity extends AppCompatActivity {

    public static final String DRONE_IP = "192.168.4.1";
    public static final int DRONE_PORT = 333;
    private ControlTask netTask;
    public boolean droneConnected;

    private Queue<String> debugQueue;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        debugQueue = new ConcurrentLinkedQueue<>();
        this.netTask = new ControlTask(this);
        this.netTask.execute(DRONE_IP, DRONE_PORT);
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent ev) {
        if (this.droneConnected) {
            ((TextView)findViewById(R.id.droneStatus)).setText("Drone connected!");

            while (!debugQueue.isEmpty()) {
                TextView debug = ((TextView) findViewById(R.id.debug));
                debug.setText(debugQueue.remove() + debug.getText());
            }
        }

        XBoxAdapter xbox = new XBoxAdapter(ev);
        ControlPacket packet = new ControlPacket(xbox.getLX(), xbox.getRY(), xbox.getRX(),
                xbox.getLY(), (byte)0);
        ((TextView)findViewById(R.id.lastPacket)).setText(packet.toString());
        this.netTask.updateControl(packet);
        return false;
    }

    @Override
    protected void onDestroy() {
        if (this.netTask != null) {
            this.netTask.cancel(true);
        }
        super.onDestroy();
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        return super.dispatchKeyEvent(event);
    }

    public void receiveControlData(Byte[] data) {

    }

    public void print(String s) {
        debugQueue.add(s);
    }
}
