package com.example.autonomous_system;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import androidx.appcompat.app.AppCompatActivity;

public class IpPortActivity extends AppCompatActivity {

    Button connectbtn;
    EditText input_ip;
    EditText input_port;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_first);

        connectbtn = (Button) findViewById(R.id.Connect);
        input_ip = (EditText) findViewById(R.id.Ip);
        input_port = (EditText) findViewById(R.id.Port);

        connectbtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(getApplicationContext(), MainActivity.class);
                String ip = input_ip.getText().toString();
                String port = input_port.getText().toString();

                intent.putExtra("ip", ip);
                intent.putExtra("port", port);
                startActivity(intent);
            }
        });
    }
}
