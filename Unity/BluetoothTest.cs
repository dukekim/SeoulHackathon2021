using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BluetoothTest : MonoBehaviour
{
    public static BluetoothTest instance;
    public Text deviceName;
    public Text dataToSend;
    public Text dataToShown;
    private bool IsConnected;
    public static string dataRecived = "";
    public string str_DeviceName = "HC-05";
    public bool isOpened = false;

    // Start is called before the first frame update
    void Awake()
    {
        if (instance == null)
            instance = this;

        IsConnected = false;
        BluetoothService.CreateBluetoothObject();
    }

    private void Start()
    {
        StartCoroutine(Cor());
    }


    // 1초마다 한번씩 확인한다.
    IEnumerator Cor()
    {
        int count = 0;
        bool isCor = true;
        while (isCor)
        {
            yield return new WaitForSeconds(1f);

            // 마스크를 인식했다면, 
            if (isOpened)
            {
                SendButton();
                WebCamAndSpinner.instance.OnPauseButtonClick();
                count++;
            }

            // 5초 후 다시 카메라를 Play상태로 바꾼다.
            if (count > 5)
            {
                WebCamAndSpinner.instance.OnPlayButtonClick();

                count = 0;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (IsConnected) {
            try
            {
               string datain =  BluetoothService.ReadFromBluetooth();
                if (datain.Length > 1)
                {
                    dataRecived = datain;
                    print(dataRecived);
                }

            }
            catch (Exception e)
            {

            }
        }
        
    }

    public void StartButton()
    {
        if (!IsConnected)
        {
            //print(deviceName.text.ToString());
            IsConnected =  BluetoothService.StartBluetoothConnection(str_DeviceName);
            if (IsConnected)
            {
                deviceName.text = str_DeviceName + " is Connected";
            }
            else
                deviceName.text = str_DeviceName + " is Disconnected";
        }
    }

    public void SendButton()
    {
        if (IsConnected && (dataToSend.ToString() != "" || dataToSend.ToString() != null))
        {
            print(dataToSend.text);

            BluetoothService.WritetoBluetooth(dataToSend.text.ToString());
            dataToShown.text = dataToSend.text;
        }
        else
        {
            BluetoothService.WritetoBluetooth("@O");
            dataToShown.text = "@O";
        }
    }


    public void StopButton()
    {
        if (IsConnected)
        {
            BluetoothService.StopBluetoothConnection();
        }
        Application.Quit();
    }
}
