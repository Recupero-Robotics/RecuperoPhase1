using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;
using System.IO;
using System.IO.Ports;

using Enablegames;

public class SerialHandler : MonoBehaviour
{
    public enum Phase
    {
        calibration,
        request,
        active
    }
    private Phase currentPhase;
    public bool fakeData = true;
    // set up serial port
    string TheraDriveState;
    string[] TheraDriveArray = new string[17];
    SerialPort serialPort;
    public SkeletonData skeletonData;
    public RoboticData roboticData; ///Biometric data from separate communication 
    public RoboticDatum roboticDatum; ///Biometric data from separate communication 
    public BiometricData biometricData; ///Biometric data from separate communication 
    public BiometricDatum biometricDatum;

    ///Biometric data from separate communication
    public int forceLevel;
    public int setPoint;
    float heartrate;
    float galvanicSkin;
    float robotAngle;
    float pinchAngle;
    public static float baselineGSR;
    public static int baselineHR;

    // General Variables/

    int theta1 = 0;
    int force1 = 0;
    int button1raw = 0;
    bool button1 = false;
    int analogX1 = 0;
    int analogY1 = 0;
    bool analogB1 = false;
    int GSR1 = new int();
    int BVP1 = new int();

    int theta2 = 0;
    int force2 = 0;
    bool button2 = false;
    int analogX2 = 0;
    int analogY2 = 0;
    bool analogB2 = false;
    int GSR2 = new int();
    int BVP2 = new int();

    private bool initialized = false;
    public static string userid = "unknownID";

    private bool gameEnded = false;

    egInt controlMode = 3;

    private bool flipState = false;

    [SerializeField, Tooltip("If the input needs to flipped for left-handed-ness.")]
    private bool flipNeed = false;

    public bool closeSerialPortAfterEachFrame = false;


    void Awake()
    {
        VariableHandler.Instance.Register(ParameterStrings.CONTROL_MODE, controlMode);
//    }

        userid = PlayerPrefs.GetString("lastUsedUserName");

        if (flipNeed && String.Equals(userid[userid.Length - 1].ToString(), "L"))
        {
            flipState = true;
        }
        else
        {
            flipState = false;
        }
    }

    void Start()
    {

        if (skeletonData == null)
        {
            GameObject go = GameObject.Find("Tracking Avatar");
            if (go != null)
                skeletonData = go.GetComponentInChildren<SkeletonData>();
        }
        if (skeletonData != null)
        {
            roboticData = skeletonData.roboticData;
            biometricData = skeletonData.biometricData;
        }
        if (roboticData == null)
            roboticData = new RoboticData();
 //       if (roboticData.data.Count <= 1)
        {
            print("SerialHandler:SET RD");
            roboticDatum = new RoboticDatum();
            roboticDatum.Value = 0f;
            roboticData.data["R1"] = roboticDatum;

            roboticDatum = new RoboticDatum();
            roboticDatum.Value = 0f;
            roboticData.data["R2"] = roboticDatum;

            roboticDatum = new RoboticDatum();
            roboticDatum.Value = 0f;
            roboticData.data["B1"] = roboticDatum;
        }

        if (biometricData == null)
            biometricData = new BiometricData();
 //       if (biometricData.data.Count <= 1)
        {
            biometricDatum = new BiometricDatum();
            biometricDatum.Value = 0f;
            biometricData.data["HR"] = biometricDatum;

            biometricDatum = new BiometricDatum();
            biometricDatum.Value = 0f;
            biometricData.data["GSR"] = biometricDatum;
        }
        if (skeletonData != null)
        {
            skeletonData.roboticData = roboticData;
            skeletonData.biometricData = biometricData;
        }

        //Start Serial Port
        serialPort = null;//new SerialPort("/dev/cu.usbmodem146101", 115200, Parity.None, 8, StopBits.One);
        if (!fakeData)
        {
            print(fakeData);
            serialPort = new SerialPort("COM6", 115200, Parity.None, 8, StopBits.One);
        }else{
            initialized = true;
        }


        //        serialPort = new SerialPort("/dev/cu.Bluetooth-Incoming-Port", 115200, Parity.None, 8, StopBits.One);
        if (serialPort != null)
            serialPort.Open();

        StartCoroutine(StartRequestingPhase());
    }

    public void StartCalibration()
    {
        currentPhase = Phase.calibration;
    }

    public void StartWriting()
    {
        currentPhase = Phase.active;
    }

    public void StartRequesting()
    {
        currentPhase = Phase.request;
    }

    void ReadSerialData()
    {

        if (serialPort == null)
            return;
        // Open the port (it's usually closed going in)
        if (!serialPort.IsOpen)
        {
            serialPort.Open();
        }

        if (initialized && gameEnded)
        {
            serialPort.Write("e" + "," + userid + "," + DateTime.UtcNow + "," + "\n");
            if (serialPort.IsOpen)
            {
                serialPort.Close();
            }
            return;
        }

        TheraDriveState = serialPort.ReadLine();
        TheraDriveArray = TheraDriveState.Split(',');

        if (TheraDriveArray.Length == 16)
        {
            initialized = true;
        }

        if (initialized)
        {
            switch (currentPhase)
            {
                case Phase.active:
                    /*
                     * WORK IN PROGRESS CODE
                     * Variable ID string: Check if data packet contains data labels
                     * if (TheraDriveArray[0]=='ID')
                     * {
                     *      // TheraDriveArray contains comma separated strings to identify variables passed from Arduino
                     *      // TheraDriveArray[1] contains the number of variables to follow
                     * }
                     * else //assign Thera Drive Variables below, as is default
                    */

                    //Assign Thera Drive Variables
                    int.TryParse(TheraDriveArray[0].ToString(), out theta1);
                    int.TryParse(TheraDriveArray[1].ToString(), out force1);
                    int.TryParse(TheraDriveArray[2].ToString(), out button1raw);
                    if (button1raw == 0)
                    {
                        button1 = true;
                    }
                    else
                    {
                        button1 = false;
                    }

                    int.TryParse(TheraDriveArray[3].ToString(), out analogX1);
                    int.TryParse(TheraDriveArray[4].ToString(), out analogY1);
                    bool.TryParse(TheraDriveArray[5].ToString(), out analogB1);
                    int.TryParse(TheraDriveArray[6].ToString(), out GSR1);
                    int.TryParse(TheraDriveArray[7].ToString(), out BVP1);


                    int.TryParse(TheraDriveArray[8].ToString(), out theta2);
                    int.TryParse(TheraDriveArray[9].ToString(), out force2);
                    bool.TryParse(TheraDriveArray[10].ToString(), out button2);
                    int.TryParse(TheraDriveArray[11].ToString(), out analogX2);
                    int.TryParse(TheraDriveArray[12].ToString(), out analogY2);
                    bool.TryParse(TheraDriveArray[13].ToString(), out analogB2);
                    int.TryParse(TheraDriveArray[14].ToString(), out GSR2);
                    int.TryParse(TheraDriveArray[15].ToString(), out BVP2);

                    /*
                     * WORK IN PROGRESS CODE
                     * Serial write to robot
                     * current syntax:
                     * outputs 1-3 are for the base robot 
                     * output 0 is for initialization state
                     * output 1 is for desired angle or other data point output to controller: three digits 
                     * output 2 is for controller magnitude/gain: two digits,  0 to 8 to scale resistance/assistance
                     * output 3 is controller mode: single digit, 0 for zero impedance, 1 for basic angular assist, 2 for basic angular resist, 3 for dynamic resistance
                     * 
                     * outputs 4-6 are for an accessory or handle with similar syntax to the Thera Drive
                     * output 4 is controller mode for accessory: single digit
                     * output 5 is controller magnitude/gain for accesory: two digits 0 to 99
                     * output 6 is for desired angle or other data point: two or three digits
                     */

                    serialPort.Write("a" + "," + Convert.ToString(setPoint) + "," + Convert.ToString(forceLevel) + "," + controlMode + "\n");
                    Debug.Log("force:" + forceLevel);
                    break;
                case Phase.request:
                    /*
                     * WORK IN PROGRESS CODE
                     * Variable ID string: Check if data packet contains data labels
                     * if (TheraDriveArray[0]=='ID')
                     * {
                     *      // TheraDriveArray contains comma separated strings to identify variables passed from Arduino
                     *      // TheraDriveArray[1] contains the number of variables to follow
                     * }
                     * else //assign Thera Drive Variables below, as is default
                    */

                    //Assign Thera Drive Variables
                    int.TryParse(TheraDriveArray[0].ToString(), out theta1);
                    int.TryParse(TheraDriveArray[1].ToString(), out force1);
                    int.TryParse(TheraDriveArray[2].ToString(), out button1raw);
                    if (button1raw == 0)
                    {
                        button1 = true;
                    }
                    else
                    {
                        button1 = false;
                    }

                    int.TryParse(TheraDriveArray[3].ToString(), out analogX1);
                    int.TryParse(TheraDriveArray[4].ToString(), out analogY1);
                    bool.TryParse(TheraDriveArray[5].ToString(), out analogB1);
                    float.TryParse(TheraDriveArray[6].ToString(), out baselineGSR);
                    baselineGSR = baselineGSR / 3;
                    int.TryParse(TheraDriveArray[7].ToString(), out baselineHR);


                    int.TryParse(TheraDriveArray[8].ToString(), out theta2);
                    int.TryParse(TheraDriveArray[9].ToString(), out force2);
                    bool.TryParse(TheraDriveArray[10].ToString(), out button2);
                    int.TryParse(TheraDriveArray[11].ToString(), out analogX2);
                    int.TryParse(TheraDriveArray[12].ToString(), out analogY2);
                    bool.TryParse(TheraDriveArray[13].ToString(), out analogB2);
                    int.TryParse(TheraDriveArray[14].ToString(), out GSR2);
                    int.TryParse(TheraDriveArray[15].ToString(), out BVP2);

                    serialPort.Write("r" + "," + Convert.ToString(setPoint) + "," + Convert.ToString(forceLevel) + "," + controlMode + "\n");

                    break;
                case Phase.calibration:
                    serialPort.Write("c" + "," + userid + "," + DateTime.Now + ":" + DateTime.Now.Millisecond + "," + Application.productName + "\n");
                    break;
            }


        }
        else
        {
            serialPort.Write("s" + "," + userid + "," + DateTime.Now + ":" + DateTime.Now.Millisecond + "," + Application.productName + "\n");
        }
        //Close port and reopen on next read

        if (serialPort.IsOpen && closeSerialPortAfterEachFrame)
        {
            serialPort.Close();
        }
    }

    void CalculateRobotAngle()
    {
        if (fakeData)
        {
            robotAngle = 100f * Mathf.Sin(Time.time);
        }
        else
        {
            robotAngle = (theta1 - 135);
            if (flipState)
            {
                robotAngle = -robotAngle;
            }
        }

        /*if (!rightState)
        {
            robotAngle = -robotAngle;
        }*/
        pinchAngle = theta2;
        print("R1=" + robotAngle);
    }

    void CalculateHeartrate()
    {
        heartrate = BVP1 + 50;
    }

    void CalculateGSR()
    {
        galvanicSkin = GSR1 / 3;
    }

    // Update is called once per frame
    void Update()
    {


        {
            if (skeletonData == null)
            {
                GameObject go = GameObject.Find("Tracking Avatar");
                if (go != null)
                    skeletonData = go.GetComponentInChildren<SkeletonData>();
                if (skeletonData != null && skeletonData.roboticData != null)
                {
                    roboticData = skeletonData.roboticData;
                }
                if (skeletonData != null && skeletonData.biometricData != null)
                {
                    biometricData = skeletonData.biometricData;
                }
            }
            if (skeletonData != null)
            {
                skeletonData.roboticData = roboticData;
                skeletonData.biometricData = biometricData;
            }

            ReadSerialData();

            if (!initialized || gameEnded)
            {
                return;
            }

            CalculateHeartrate();
            CalculateRobotAngle();
            print("SH:Up: R1= " + robotAngle);
            roboticData.SetData("R1", robotAngle, Time.time);
            roboticData.SetData("B1", button1raw, Time.time);
            roboticData.SetData("R2", pinchAngle, Time.time);
            biometricData.SetData("HR", heartrate, Time.time);
            biometricData.SetData("GSR", galvanicSkin, Time.time);

            Debug.Log("GSR: " + GSR1);


            if (Input.GetKeyDown(KeyCode.E) && initialized && !gameEnded)
            {
                GameEnded();
            }
        }
    }

    void OnDestroy()
    {
        if (fakeData)
            return;
        Debug.Log("Port Closed");
        if (!serialPort.IsOpen)
        {
            serialPort.Open();
        }
        serialPort.Write("e" + "," + userid + "," + DateTime.UtcNow + "," + "\n");
        if (serialPort.IsOpen)
        {
            serialPort.Close();
        }

    }
    public void GameEnded()
    {
        gameEnded = true;
    }

    public void Restart()
    {
        gameEnded = false;
        initialized = false;
    }

    IEnumerator StartRequestingPhase()
    {
        StartRequesting();
        yield return new WaitForSeconds(1);
        StartWriting();
    }
}
