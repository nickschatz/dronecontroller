package com.nickschatz.dronecontrol;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    public static final String DRONE_IP = "192.168.4.1";
    public static final int DRONE_PORT = 333;
    private ControlTask netTask;
    public boolean droneConnected;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.netTask = new ControlTask(this);
        this.netTask.execute(DRONE_IP, DRONE_PORT);
    }

    @Override
    public boolean dispatchGenericMotionEvent(MotionEvent ev) {
        if (this.droneConnected) {
            ((TextView)findViewById(R.id.droneStatus)).setText("Drone connected!");
        }

        XBoxAdapter xbox = new XBoxAdapter(ev);
        ControlPacket packet = new ControlPacket(xbox.getLX(), xbox.getLY(), xbox.getRY(), (byte)0);
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
}
