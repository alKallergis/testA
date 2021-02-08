package com.example.android.JSON_test_W_cc3200v2_1;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.net.NetworkInfo;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import org.json.JSONException;
import org.json.JSONObject;

import java.net.DatagramSocket;

public class MainActivity extends AppCompatActivity {
    WifiManager wifimngr;
    TextView statusTextview;
    TextView rssiView;
    TextView distView;
    EditText mStartFreqView;
    EditText mEndFreqView;
    EditText mFreqStepView;
    String ssid="cctestAP";
    WifiConfiguration wifiConfig;
    WifiInfo wifiInfo;
    DatagramSocket s = null;
    ConnectTask connectTask1;
    TcpClient mTcpClient;
    private Button sendButton;
    private Spinner mWaveformSpinner;
    private int waveform=1;//sine default

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.d(null, "onCreate");
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);       //activity keeps screen on
        sendButton = findViewById(R.id.sendbutton);
        mWaveformSpinner=(Spinner) findViewById(R.id.spinner_waveform) ;
        statusTextview = (TextView) findViewById(R.id.statusV);
        rssiView = (TextView) findViewById(R.id.rssiview);
        distView = (TextView) findViewById(R.id.distview);
        mStartFreqView=(EditText)  findViewById(R.id.startFreq);
        mEndFreqView=(EditText) findViewById(R.id.endFreq);
        mFreqStepView=(EditText)    findViewById(R.id.stepFreq) ;

        wifiConfig = new WifiConfiguration();
        wifiConfig.SSID = String.format("\"%s\"", ssid);
        //wifiConfig.preSharedKey = String.format("\"%s\"", key);       //this is an open network
        setupSpinner();

        sendButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //sends the message to the server
                try {
                    int IntStartFreq=Integer.parseInt(mStartFreqView.getText().toString());
                    int IntEndFreq=Integer.parseInt(mEndFreqView.getText().toString());
                    int IntStepFreq=Integer.parseInt(mFreqStepView.getText().toString());
                    if ((IntStartFreq>9)&&(IntStartFreq<4000)&&(IntEndFreq>IntStartFreq)&&(IntEndFreq<4001)&&(IntStepFreq > 0)) {
                        JSONObject sweep1 = new JSONObject();
                        try {
                            sweep1.put("waveform", String.valueOf(waveform));
                            sweep1.put("startFreq", mStartFreqView.getText().toString().trim());
                            sweep1.put("endFreq", mEndFreqView.getText().toString().trim());
                            sweep1.put("stepFreq", mFreqStepView.getText().toString().trim());

                        } catch (JSONException e) {
                            e.printStackTrace();
                            Toast.makeText(getApplicationContext(), e.toString(), Toast.LENGTH_SHORT).show();
                        }
                        mTcpClient.sendMessage(sweep1.toString());
                    }
                    else{
                        Toast.makeText(getApplicationContext(), "Wrong values", Toast.LENGTH_SHORT).show();
                    }
                }
                catch (Exception e) {
                    e.printStackTrace();
                    Toast.makeText(getApplicationContext(), e.toString(), Toast.LENGTH_SHORT).show();
                }


            }
        });
    }
        @Override
    protected void onResume() {
        Log.d(null,"onResume");
        super.onResume();
        IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(WifiManager.NETWORK_STATE_CHANGED_ACTION);
        intentFilter.addAction(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION);
        intentFilter.addAction(WifiManager.RSSI_CHANGED_ACTION);
        registerReceiver(wifistateReceiver, intentFilter);
        wifimngr = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        wifimngr.startScan();       //start a scan anyway on app start

}

    private BroadcastReceiver wifistateReceiver=new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            wifimngr = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            wifiInfo = wifimngr.getConnectionInfo();
            final String action = intent.getAction();
            if (action.equals(WifiManager.NETWORK_STATE_CHANGED_ACTION)) {
                if (intent.getExtras() != null) {
                    NetworkInfo ni = (NetworkInfo) intent.getExtras().get(WifiManager.EXTRA_NETWORK_INFO);
                    String APname=ni.getExtraInfo().replace("\"", "");
                    if (ni != null && ni.getState() == NetworkInfo.State.CONNECTED) {
                        Log.d("app", "Network " + ni.getTypeName() + " connected");
                        statusTextview.setText("CONNECTED TO CC3200");
                        if (wifiInfo.getIpAddress() != 0 &&APname.equals(ssid)) {       //check if ip has been obtained and we are in correct network
                            if (connectTask1 ==null|| connectTask1.isCancelled()){      //ensure only 1 task is running
                                connectTask1 = (ConnectTask) new ConnectTask();
                                connectTask1.execute("testkey");}
                        }
                        else{
                            statusTextview.setText("NOT CONNECTED TO CC3200");
                            rssiView.setText("");
                            distView.setText("");
                            if (connectTask1 != null&&!connectTask1.isCancelled()) {  //if recvtask has been instantiated AND is not cancelled,cancel it
                                connectTask1.cancel(true);
                                Log.d(null,"thread stopped in broadcast receiver/network state connected to other AP");
                            }
                        }
                    }
                    else{
                        if (connectTask1 != null&&!connectTask1.isCancelled()) {
                            connectTask1.cancel(true);
                            Log.d(null,"thread stopped in broadcast receiver/network state changed/disconnected");
                        }
                        statusTextview.setText("NO WIFI"); //TODO: CHANGE THIS TO NO WIFI/NOT CONNECTED
                        rssiView.setText("");
                        distView.setText("");}
                }
            }
            if (action.equals((WifiManager.SCAN_RESULTS_AVAILABLE_ACTION))) {       //this is the receiver for scan results. if connected,scans can only be invoked by startscan method.
/*                    List<ScanResult> sr = ((WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE)).getScanResults();      //will this get the new results??
                    for (int i = 0; i < sr.size(); i++) {
                        String info = ((sr.get(i)).toString());
                        Log.d("app", "result"+i+":" + info);
                    }*/
                if(!wifimngr.getConnectionInfo().getSSID().replace("\"", "").equals(ssid)){   //try to connect to the correct network
                    //remember id
                    int netId = wifimngr.addNetwork(wifiConfig);
                    wifimngr.disconnect();
                    wifimngr.enableNetwork(netId, true);
                    wifimngr.reconnect();}
            }
            if(action.equals(WifiManager.RSSI_CHANGED_ACTION)){
                Log.d(null,"new rssi");
                if (wifiInfo.getSSID().replace("\"", "").equals("cctestAP")) {
                    int rssi = intent.getIntExtra(WifiManager.EXTRA_NEW_RSSI, -100);
                    rssiView.setText("RSSI: " + rssi + "dBm");      //Distance (km) = 10(Free Space Path Loss – 32.44 – 20log10(f))/20
                    float f = (float) DistCalc.calculateDistance(rssi, 2447);       //channel 8 = 2447mhz
                    distView.setText("distance ~" + String.format("%.2f", f)+"m");
                }
            }
        }
    };

    public class ConnectTask extends AsyncTask<String, String, TcpClient> {

        @Override
        protected TcpClient doInBackground(String... message) {
            Log.d(null, "new Thread started");
            synchronized (this) {
//                while (!Thread.interrupted()) {       //we won't use this. Let task end and re-start it when connected. If reconnection takes too long,user may need to connect manually or restart app

                try {
                    //we create a TCPClient object
                    mTcpClient = new TcpClient(new TcpClient.OnMessageReceived() {
                        @Override
                        //here the messageReceived method is implemented
                        public void messageReceived(String message) {
                            //this method calls the onProgressUpdate
                            publishProgress(message);
                        }
                    });
                    mTcpClient.run();


                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    if (mTcpClient != null) {
                        mTcpClient.stopClient();
                    }
                    return null;
                }
            }

 /*                   try{
                    Thread.sleep(1000);}
                    catch (Exception e2){
                        Log.d(null, "EXCEPTION2 :" + e2.getMessage());
                    }
                }*/
        }

        @Override
        protected void onProgressUpdate(String... values) {
            super.onProgressUpdate(values);
            //response received from server
            Log.d("test", "response " + values[0]);
            //process server response here....
            statusTextview.setText("Status= " + values[0]);

        }

        @Override
        protected void onPostExecute(TcpClient tcpClient) {
            Log.d(null,"onPostexecute");
            super.onPostExecute(tcpClient);
            statusTextview.setText("NOT CONNECTED TO CC3200");
            rssiView.setText("");
            distView.setText("");
            wifimngr.startScan();     //start a new scan??
        }

        @Override
        protected void onCancelled() {
            Log.d(null,"onCancelled");
            super.onCancelled();
/*          if (s != null) {
                s.close();
                Log.d(null,"closed in onCancelled");    //IS THIS NEEDED? SEEMS LIKE TASK THROWS INTERRUPTED-EXCEPTION ANYWAY..
            }*/
        }
    }

    @Override
    protected void onPause() {
        Log.d(null,"onPause");
        super.onPause();
        unregisterReceiver(wifistateReceiver);
        if (mTcpClient != null) {
            mTcpClient.stopClient();
        }
        if ((connectTask1 != null)&&(!connectTask1.isCancelled())) {
            connectTask1.cancel(true);
            Log.d(null,"thread stopped in onPause");
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.d(null,"onDestroy");
/*        if (recvTask1 != null) {
            recvTask1.cancel(true);
            Log.d(null,"thread stopped in onDestroy");
        }*/
    }

    //this is for viewing a readable ip..
    public String getIpAddr() {
        WifiManager wifiManager = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
        WifiInfo wifiInfo = null;
        if (wifiManager != null) {
            wifiInfo = wifiManager.getConnectionInfo();
        }
        int ip = 0;
        if (wifiInfo != null) {
            ip = wifiInfo.getIpAddress();
        }

        String ipString = String.format(
                "%d.%d.%d.%d",
                (ip & 0xff),
                (ip >> 8 & 0xff),
                (ip >> 16 & 0xff),
                (ip >> 24 & 0xff));

        return ipString;
    }

    /**
     * Setup the dropdown spinner that allows the user to select the waveform type.
     */
    private void setupSpinner() {
        // Create adapter for spinner. The list options are from the String array it will use
        // the spinner will use the default layout
        ArrayAdapter genderSpinnerAdapter = ArrayAdapter.createFromResource(this,
                R.array.array_waveform_options, android.R.layout.simple_spinner_item);

        // Specify dropdown layout style - simple list view with 1 item per line
        genderSpinnerAdapter.setDropDownViewResource(android.R.layout.simple_dropdown_item_1line);

        // Apply the adapter to the spinner
        mWaveformSpinner.setAdapter(genderSpinnerAdapter);

        // Set the integer mSelected to the constant values
        mWaveformSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String selection = (String) parent.getItemAtPosition(position);
                if (!TextUtils.isEmpty(selection)) {
                    if (selection.equals("sinusoid")) {
                        waveform = 1;
                    } else if (selection.equals("triangle wave")) {
                        waveform = 2;
                    }else if (selection.equals("square wave")){
                        waveform=3;
                    }else {
                        waveform = 1;
                    }
                }
            }

            // Because AdapterView is an abstract class, onNothingSelected must be defined
            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                waveform = 1;
            }
        });
    }

}