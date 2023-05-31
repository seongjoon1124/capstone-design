package com.example.autonomous_system;

import androidx.appcompat.app.AppCompatActivity;

import android.os.AsyncTask;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.content.Intent;
import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;

public class MainActivity extends AppCompatActivity {

    Button upbtn;
    Button downbtd;
    Button leftbtn;
    Button rightbtn;
    Button autobtn;
    Button manualbtn;
    Button stopbtn;

    int port_i;
    String ip_s;
    String port_s;
    Socket socket = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Intent intent = getIntent();


        ip_s = intent.getStringExtra("ip");
        port_s = intent.getStringExtra("port");
        port_i = Integer.parseInt(port_s);

        upbtn = (Button) findViewById(R.id.Up);
        downbtd = (Button) findViewById(R.id.Down);
        leftbtn = (Button) findViewById(R.id.Left);
        rightbtn = (Button) findViewById(R.id.Right);
        autobtn = (Button) findViewById(R.id.Auto);
        manualbtn = (Button) findViewById(R.id.Manual);
        stopbtn = (Button) findViewById(R.id.Stop);

        autobtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ConnectTask c = new ConnectTask(ip_s, port_i, "auto");
                c.execute();
            }
        });

        manualbtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ConnectTask c = new ConnectTask(ip_s, port_i, "manual");
                c.execute();
            }
        });

        stopbtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ConnectTask c = new ConnectTask(ip_s, port_i, "stop");
                c.execute();
            }
        });

        upbtn.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "up");
                    c.execute();
                }
                else if(event.getAction() == MotionEvent.ACTION_UP){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "stop");
                    c.execute();
                }
                return false;
            }
        });

        downbtd.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "down");
                    c.execute();
                }
                else if(event.getAction() == MotionEvent.ACTION_UP){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "stop");
                    c.execute();
                }
                return false;
            }
        });

        leftbtn.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "left");
                    c.execute();
                }
                else if(event.getAction() == MotionEvent.ACTION_UP){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "stop");
                    c.execute();
                }
                return false;
            }
        });

        rightbtn.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "right");
                    c.execute();
                }
                else if(event.getAction() == MotionEvent.ACTION_UP){
                    ConnectTask c = new ConnectTask(ip_s, port_i, "stop");
                    c.execute();
                }
                return false;
            }
        });


    }



    public class ConnectTask extends AsyncTask<Void, Void, Void> {
        String dstAddress;
        int dstPort;
        String myMessage = "";

        //constructor
        ConnectTask(String addr, int port, String message) {
            dstAddress = addr;
            dstPort = port;
            myMessage = message;
        }

        @Override
        protected Void doInBackground(Void... arg0) {
            Socket socket = null;
            myMessage = myMessage.toString();
            try {
                socket = new Socket(dstAddress, dstPort);
                //송신
                OutputStream out = socket.getOutputStream();
                out.write(myMessage.getBytes());
            } catch (IOException e) {
                e.printStackTrace();
            }
            return null;
        }
    }
}

