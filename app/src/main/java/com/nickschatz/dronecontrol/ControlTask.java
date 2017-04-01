package com.nickschatz.dronecontrol;

import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.Collections;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import static android.R.id.input;

public class ControlTask extends AsyncTask<Object, Byte, Void> {

    private final MainActivity main;
    private Socket socket;
    private Queue<ControlPacket> ctrlData;
    private Queue<Byte> recvQueue;
    private boolean stayAlive = true;

    public ControlTask(MainActivity ente) {
        ctrlData = new ConcurrentLinkedQueue<>();
        this.main = ente;
    }

    @Override
    protected Void doInBackground(Object[] params) {
        String ip = (String) params[0];
        int port = (int) params[1];
        while (this.socket == null) {
            try {
                this.socket = new Socket(ip, port);
            } catch (IOException e) {
                // Failed to connect
                Log.d("Control Task", "Failed to connect...", e);
            }
        }
        Log.d("Control Task", "Connected!");
        this.main.droneConnected = true;

        try (InputStream input = this.socket.getInputStream();
            OutputStream output = this.socket.getOutputStream()) {
            ControlPacket latest = null;
            while (this.stayAlive) {
                while (!ctrlData.isEmpty()) {
                    latest = ctrlData.remove();
                }
                if (latest != null) {
                    output.write(latest.pack());
                }
                while (input.available() > 0) {
                    recvQueue.add((byte) input.read());
                }
                Thread.sleep(50);
            }
            input.close();
            output.close();
        }
        catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }

        return null;
    }

    public void updateControl(ControlPacket ctrl) {
        ctrlData.add(ctrl);
    }

    @Override
    protected void onProgressUpdate(Byte... values) {
        main.receiveControlData(values);
    }
}