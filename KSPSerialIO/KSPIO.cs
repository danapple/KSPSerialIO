
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Reflection;
using System.Threading;
using Microsoft.Win32;
using System.Runtime.InteropServices;

using Psimax.IO.Ports;
using UnityEngine;
using KSP.IO;
using KSP.UI.Screens;

namespace KSPSerialIO
{
    #region Structs
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct VesselData
    {
        public byte id;             
        public byte ackseq;         
        public float AP;            
        public float PE;            
        public float SemiMajorAxis; 
        public float SemiMinorAxis; 
        public float VVI;           
        public float e;             
        public float inc;           
        public float G;             
        public int TAp;             
        public int TPe;             
        public float TrueAnomaly;   
        public float Density;       
        public int period;          
        public float RAlt;          
        public float Alt;           
        public float Vsurf;         
        public float Lat;           
        public float Lon;           
        public float LiquidFuelTot; 
        public float LiquidFuel;    
        public float OxidizerTot;   
        public float Oxidizer;      
        public float EChargeTot;    
        public float ECharge;       
        public float MonoPropTot;   
        public float MonoProp;      
        public float IntakeAirTot;  
        public float IntakeAir;     
        public float SolidFuelTot;  
        public float SolidFuel;     
        public float XenonGasTot;   
        public float XenonGas;      
        public float LiquidFuelTotS;
        public float LiquidFuelS;   
        public float OxidizerTotS;  
        public float OxidizerS;     
        public UInt32 MissionTime;  
        public float deltaTime;     
        public float VOrbit;        
        public UInt32 MNTime;       
        public float MNDeltaV;      
        public float Pitch;         
        public float Roll;          
        public float Heading;       
        public UInt16 ActionGroups; //  status bit order:SAS, RCS, Light, Gear, Brakes, Abort, Custom01 - 10 
        public byte SOINumber;      //  SOI Number (decimal format: sun-planet-moon e.g. 130 = kerbin, 131 = mun)
        public byte MaxOverHeat;    //  Max part overheat (% percent)
        public float MachNumber;    //
        public float IAS;           //  Indicated Air Speed
        public byte CurrentStage;   //  Current stage number
        public byte TotalStage;     //  TotalNumber of stages
        public float TargetDist;    //  Distance to targeted vessel (m)
        public float TargetV;       //  Target vessel relative velocity (m/s)
        public byte NavballSASMode; //  Combined byte for navball target mode and SAS mode
        // First four bits indicate AutoPilot mode:
        // 0 SAS is off  //1 = Regular Stability Assist //2 = Prograde
        // 3 = RetroGrade //4 = Normal //5 = Antinormal //6 = Radial In
        // 7 = Radial Out //8 = Target //9 = Anti-Target //10 = Maneuver node
        // Last 4 bits set navball mode. (0=ignore,1=ORBIT,2=SURFACE,3=TARGET)
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct HandShakePacket
    {
        public byte id;
        public byte M1;
        public byte M2;
        public byte M3;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct LogPacket
    {
      public byte id;
      public char[] message;
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ControlPacket
    {
        public byte id;
        public byte seq;
        public byte MainControls;                  //SAS RCS Lights Gear Brakes Precision Abort Stage 
        public byte Mode;                          //0 = stage, 1 = docking, 2 = map
        public ushort ControlGroup;                //control groups 1-10 in 2 bytes
        public byte NavballSASMode;                //AutoPilot mode (See above for AutoPilot modes)(Ignored if the equal to zero or out of bounds (>10)) //Navball mode
        public byte AdditionalControlByte1;
        public short Pitch;                        //-1000 -> 1000
        public short Roll;                         //-1000 -> 1000
        public short Yaw;                          //-1000 -> 1000
        public short TX;                           //-1000 -> 1000
        public short TY;                           //-1000 -> 1000
        public short TZ;                           //-1000 -> 1000
        public short WheelSteer;                   //-1000 -> 1000
        public short Throttle;                     // 0 -> 1000
        public short WheelThrottle;                // 0 -> 1000
    };

    public struct VesselControls
    {
        public Boolean SAS;
        public Boolean RCS;
        public Boolean Lights;
        public Boolean Gear;
        public Boolean Brakes;
        public Boolean Precision;
        public Boolean Abort;
        public Boolean Stage;
        public int Mode;
        public int SASMode;
        public int SpeedMode;
        public Boolean[] ControlGroup;
        public float Pitch;
        public float Roll;
        public float Yaw;
        public float TX;
        public float TY;
        public float TZ;
        public float WheelSteer;
        public float Throttle;
        public float WheelThrottle;
    };

    public struct IOResource
    {
        public float Max;
        public float Current;
    }

    #endregion

    enum enumAG : int
    {
        SAS,
        RCS,
        Light,
        Gear,
        Brakes,
        Abort,
        Custom01,
        Custom02,
        Custom03,
        Custom04,
        Custom05,
        Custom06,
        Custom07,
        Custom08,
        Custom09,
        Custom10,
    };

    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class SettingsNStuff : MonoBehaviour
    {
        public static PluginConfiguration cfg = PluginConfiguration.CreateForType<SettingsNStuff>();
        public static string DefaultPort;
        public static int HandshakeDelay;
        public static int HandshakeDisable;
        public static int BaudRate;
        // Throttle and axis controls have the following settings:
        // 0: The internal value (supplied by KSP) is always used.
        // 1: The external value (read from serial packet) is always used.
        // 2: If the internal value is not zero use it, otherwise use the external value.
        // 3: If the external value is not zero use it, otherwise use the internal value.        
        public static int PitchEnable;
        public static int RollEnable;
        public static int YawEnable;
        public static int TXEnable;
        public static int TYEnable;
        public static int TZEnable;
        public static int WheelSteerEnable;
        public static int ThrottleEnable;
        public static int WheelThrottleEnable;
        public static double SASTol;

        void Awake()
        {
	  //cfg["refresh"] = 0.08;
            //cfg["DefaultPort"] = "COM1";
            //cfg["HandshakeDelay"] = 2500;
            print("KSPSerialIO: Loading settings...");

            cfg.load();
            DefaultPort = cfg.GetValue<string>("DefaultPort");
            print("KSPSerialIO: Default Port = " + DefaultPort);

	    //            refreshPeriod = cfg.GetValue<double>("refresh");
	    //            print("KSPSerialIO: RefreshPeriod = " + refreshPeriod.ToString());

            BaudRate = cfg.GetValue<int>("BaudRate");
            print("KSPSerialIO: BaudRate = " + BaudRate.ToString());

            HandshakeDelay = cfg.GetValue<int>("HandshakeDelay");
            print("KSPSerialIO: Handshake Delay = " + HandshakeDelay.ToString());

            HandshakeDisable = cfg.GetValue<int>("HandshakeDisable");
            print("KSPSerialIO: Handshake Disable = " + HandshakeDisable.ToString());

            PitchEnable = cfg.GetValue<int>("PitchEnable");
            print("KSPSerialIO: Pitch Enable = " + PitchEnable.ToString());

            RollEnable = cfg.GetValue<int>("RollEnable");
            print("KSPSerialIO: Roll Enable = " + RollEnable.ToString());

            YawEnable = cfg.GetValue<int>("YawEnable");
            print("KSPSerialIO: Yaw Enable = " + YawEnable.ToString());

            TXEnable = cfg.GetValue<int>("TXEnable");
            print("KSPSerialIO: Translate X Enable = " + TXEnable.ToString());

            TYEnable = cfg.GetValue<int>("TYEnable");
            print("KSPSerialIO: Translate Y Enable = " + TYEnable.ToString());

            TZEnable = cfg.GetValue<int>("TZEnable");
            print("KSPSerialIO: Translate Z Enable = " + TZEnable.ToString());

            WheelSteerEnable = cfg.GetValue<int>("WheelSteerEnable");
            print("KSPSerialIO: Wheel Steering Enable = " + WheelSteerEnable.ToString());

            ThrottleEnable = cfg.GetValue<int>("ThrottleEnable");
            print("KSPSerialIO: Throttle Enable = " + ThrottleEnable.ToString());

            WheelThrottleEnable = cfg.GetValue<int>("WheelThrottleEnable");
            print("KSPSerialIO: Wheel Throttle Enable = " + WheelThrottleEnable.ToString());

            SASTol = cfg.GetValue<double>("SASTol");
            print("KSPSerialIO: SAS Tol = " + SASTol.ToString());
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPSerialPort : MonoBehaviour
    {
      public static System.Diagnostics.Stopwatch stopwatch;
        public static SerialPort Port;
	private static int openingVersion = 0;
        public static string PortName;
        public static Boolean DisplayFound = false;
        public static Boolean ControlReceived = false;
	public static int sendCount = 0;
	public static int rcvCount = 0;

        public static VesselData VData;
        public static ControlPacket CPacket;
        public static LogPacket LPacket;
        private static HandShakePacket SendingHPacket = new HandShakePacket();
        private static HandShakePacket ReceivedHPacket;

        public static VesselControls VControls = new VesselControls();

        private const int MaxPayloadSize = 255;
        enum ReceiveStates: byte {
            FIRSTHEADER, // Waiting for first header
            SECONDHEADER, // Waiting for second header
            SIZE, // Waiting for payload size
            PAYLOAD, // Waiting for rest of payload
            CS // Waiting for checksum
        }
        private static ReceiveStates CurrentState = ReceiveStates.FIRSTHEADER;
        private static byte CurrentPacketLength;
        private static byte CurrentBytesRead;
        // Guards access to data shared between threads
        private static Mutex SerialMutex = new Mutex();
        // Serial worker uses this buffer to read bytes
        private static byte[] PayloadBuffer = new byte[MaxPayloadSize];
        // Buffer for sharing packets from serial worker to main thrad
        private static volatile bool NewPacketFlag = false;
        private static volatile byte[] NewPacketBuffer = new byte[MaxPayloadSize];

        private static Thread SerialThread;
	public static bool newVData = false;
	
        private const byte HSPid = 77, VDid = 1, Cid = 101; //hard coded values for packet IDS
	private const byte Lid = 200;
	
        void Awake()
        {
	  stopwatch = System.Diagnostics.Stopwatch.StartNew();

	  String version = System.Reflection.Assembly.GetExecutingAssembly()
	    .GetName().Version.ToString();
	  DebugLog(String.Format("KSPSerialIO: Version {0}", version));
	  DebugLog("KSPSerialIO: Getting serial ports...");
	  DebugLog(String.Format("KSPSerialIO: Output packet size: {0}/{1}",
				  Marshal.SizeOf(VData).ToString(),
				  MaxPayloadSize));
	  initializeDataPackets();
	  if (SettingsNStuff.HandshakeDisable == 1)
	  {
	    DisplayFound = true;
	  }
	}
	
	private long lastPortTry = 0;
        private void Update()
        {
	  long nowTime = stopwatch.ElapsedMilliseconds;
	  //	  DebugLog("KSPSerialIO: KSPSerialPort.Update(), nowTime = " + nowTime);
	  if (!DisplayFound && isPortOpen(Port) && nowTime - openedAt > 5000)
	  {
	    DebugLog("KSPSerialIO: handshake timeout, closing port");
	    PortCleanup();
	  }
	  else if (DisplayFound && nowTime - lastPacket > 10000)
	  {
	    DebugLog("KSPSerialIO: packet timeout, closing port");
	    PortCleanup();
	  }
	  if (!isPortOpen(Port))
	  {
	    if (SettingsNStuff.HandshakeDisable == 0)
	    {
	      DisplayFound = false;
	    }
	    //	      DebugLog("KSPSerialIO: serial port not open");
	    if (nowTime - lastPortTry > 1000)
	    {
	      lastPortTry = nowTime;
	      openPort();
	    }
	  }
	  else
	  {
	    if (NewPacketFlag)
	    {
	      InboundPacketHandler();
	    }
	    try
	    {
	      if (!DisplayFound || nowTime - lastPacket > 5000)
	      {
		handshake();
	      }
	      if (DisplayFound && newVData)
	      {
		//DebugLog("KSPSerialIO: Sending VData with ackSeq = " + VData.ackseq);
		KSPSerialPort.sendPacket(VData);
		newVData = false;		
	      }
	    }
	    catch (Exception e)
	    {
	      DebugLog("KSPSerialIO: Error sending packet " + Port.PortName + ": " + e.Message);
	      PortCleanup();
	    }
	  }
	}

	public static bool isPortOpen(SerialPort port)
	{
	  return port != null && port.IsOpen;
	}

	private static void handshake()
	{
	  long now = stopwatch.ElapsedMilliseconds;

	  if (now - lastHandshake > 500)
	  {
	    lastHandshake = now;
	    SendingHPacket.id = HSPid;
	    SendingHPacket.M1 = 0x51;
	    SendingHPacket.M2 = 0x20;
	    SendingHPacket.M3 = 3;
	    DebugLog("KSPSerialIO: Handshake sending : " + SendingHPacket.M1.ToString() + SendingHPacket.M2.ToString() + SendingHPacket.M3.ToString());
	    sendPacket(SendingHPacket);
	  }
	}
		
	private long openedAt = 0;
        private void openPort()
        {
	  PortName = SettingsNStuff.DefaultPort;
	    
	  Port = new SerialPort(PortName, SettingsNStuff.BaudRate, Parity.Odd, 8, StopBits.Two);
	  openingVersion++;
	  
	  DebugLog("KSPSerialIO: trying port " + PortName);
	    
	  try
	  {
	    Port.Open();
	    if (isPortOpen(Port))
	    {
	      //	      byte[] closeBootloaderPacket = new byte[2];
	      //	      closeBootloaderPacket[0] = 0x51;
	      //	      closeBootloaderPacket[1] = 0x20;
	      //	      for (int i = 0; i < 100; i++)
	      //	      {
	      //		Port.Write(closeBootloaderPacket, 0, closeBootloaderPacket.Length);
	      //	      }

	      DebugLog("KSPSerialIO: port " + PortName + " open = " + isPortOpen(Port));
	      openedAt = stopwatch.ElapsedMilliseconds;
	      lastPacket = openedAt + 3000; // On host systems with DTR handling, leave time for Arduino bootloader to exit, sometimes twice
	      lastHandshake = lastPacket;
	      CurrentState = ReceiveStates.FIRSTHEADER;
	      NewPacketFlag = false;
	      startThread();
	    }
	    else
	    {
	      DebugLog("KSPSerialIO: port " + PortName + " not open, closing and disposing");
	      PortCleanup();
	    }
		
	  }
	  catch (Exception e)
	  {
	    DebugLog("KSPSerialIO: Error opening serial port " + Port.PortName + ": " + e.Message);
	  }
        }

	public static void DebugLog(string text)
	{
	  DateTime nowDateTime = DateTime.Now;
	  Debug.Log("KSPSerialIO: " + nowDateTime.ToString("{MM/dd/yy HH:mm:ss.fff zzz}") + " " + text);
	}

	private static long lastPacket = 0;
	private static long lastHandshake = 0;

        unsafe public static void InboundPacketHandler()
        {
	  //            DebugLog("KSPSerialIO: InboundPacketHandler start");
	  lastPacket = stopwatch.ElapsedMilliseconds;
            SerialMutex.WaitOne();
            NewPacketFlag = false;
            switch (NewPacketBuffer[0])
            {
                case HSPid:
		  //DebugLog("KSPSerialIO: HSPid");
                    ReceivedHPacket = (HandShakePacket)ByteArrayToStructure(NewPacketBuffer, ReceivedHPacket);
                    SerialMutex.ReleaseMutex();
                    HandShake();
                    if ((ReceivedHPacket.M1 == 3) && (ReceivedHPacket.M2 == 1) && (ReceivedHPacket.M3 == 4))
		    {
		      if (DisplayFound == false)
		      {
			DebugLog("KSPSerialIO: DisplayFound true");
		      }
		      DisplayFound = true;
                    }
		    else
                    {
		      DebugLog("KSPSerialIO: DisplayFound false");
		      if (SettingsNStuff.HandshakeDisable == 0)
		      {
			DisplayFound = false;
		      }
                    }
                    break;
                case Cid:
		  //                    DebugLog("KSPSerialIO: Cid");
                    CPacket = (ControlPacket)ByteArrayToStructure(NewPacketBuffer, CPacket);
                    SerialMutex.ReleaseMutex();
                    VesselControls();
                    break;
                case Lid:
		  // 		  DebugLog("KSPSerialIO: HostLog message");
		  //		  LPacket = (LogPacket)ByteArrayToStructure(NewPacketBuffer, LPacket);
		  char[] message = new char[201];
		  for (int i = 0; i < 200; i++)
		  {
		    message[i] = (char)NewPacketBuffer[1 + i];
		  }
		  SerialMutex.ReleaseMutex();
		  /*
		  fixed (LogPacket* lp = &LPacket)
		  {
		    for (int i = 0; i < 200; i++)
		    {
		      message[i] = lp->message[i];
		    }
		    }*/
		  message[200] = '\0';
		  DebugLog("KSPSerialIO: Arduino log: " + new string(message));
		  break;
                default:
                    SerialMutex.ReleaseMutex();
                    DebugLog("KSPSerialIO: Packet id " + NewPacketBuffer[0] + " unimplemented");
                    break;
            }
//            DebugLog("KSPSerialIO: InboundPacketHandler done");

        }

        public static void sendPacket(object anything)
        {
	  //	  DebugLog("KSPSerialIO: sendPacket start " + ++sendCount);

            byte[] Payload = StructureToByteArray(anything);
            byte header1 = 0xBE;
            byte header2 = 0xEF;
            byte size = (byte)Payload.Length;
            byte checksum = size;

            byte[] Packet = new byte[size + 4];

            //Packet = [header][size][payload][checksum];
            //Header = [Header1=0xBE][Header2=0xEF]
            //size = [payload.length (0-255)]

            for (int i = 0; i < size; i++)
            {
                checksum ^= Payload[i];
            }

            Payload.CopyTo(Packet, 3);
            Packet[0] = header1;
            Packet[1] = header2;
            Packet[2] = size;
            Packet[Packet.Length - 1] = checksum;

//            DebugLog("KSPSerialIO: about to write " + (byte)Packet.Length + " bytes");
            Port.Write(Packet, 0, Packet.Length);
//            DebugLog("KSPSerialIO: sendPacket done " + sendCount);
        }

	private void startThread()
	{
            SerialThread = new Thread(SerialWorker);
            SerialThread.Start();
	}

	public static void abortThread()
	{
	  if (SerialThread != null)
	  {
	    SerialThread.Abort();
	  }
	}

        private void SerialWorker()
        {
	  int myOpeningVersion = openingVersion;
	  byte[] buffer = new byte[MaxPayloadSize + 4];
	  DebugLog("KSPSerialIO: Serial Worker thread " + myOpeningVersion + " started");
	  SerialPort readPort = Port;

	  bool keepRunning = true;
	  while (keepRunning)
          {
	    readPort = Port;
	    keepRunning &= myOpeningVersion == openingVersion && isPortOpen(readPort);
	    if (keepRunning)
	    {
	      try
              {
		//DebugLog("KSPSerialIO: trying to read in try");
		int actualLength = readPort.Read(buffer, 0, buffer.Length);
		if (actualLength < 1)
		{
		  DebugLog("KSPSerialIO: SerialWorker EOF");
		  break;
		}
		//DebugLog("KSPSerialIO: Got " + actualLength + " bytes of data");
		byte[] received = new byte[actualLength];
		Buffer.BlockCopy(buffer, 0, received, 0, actualLength);
		ReceivedDataEvent(received, actualLength);
	      }
	      catch (Exception e)
              {
		DebugLog("KSPSerialIO: Serial Worker thread " + myOpeningVersion + " failed reading port: " + e.Message);
		keepRunning = false;
	      }
	    }
	  }
	  DebugLog("KSPSerialIO: Serial worker " + myOpeningVersion + " thread shutting down.");
        }

        private void ReceivedDataEvent(byte[] ReadBuffer, int BufferLength)
        {
	  //	    DebugLog("KSPSerialIO: ReceivedDataEvent start with " + BufferLength + " bytes, CurrentState = " + CurrentState);
            for (int x=0; x<BufferLength; x++)
            {
	      //	      DebugLog("KSPSerialIO: Starting in state " + CurrentState + " with byte 0x" + BitConverter.ToString(ReadBuffer[x])
	      //			  + ", CurrentBytesRead = " + CurrentBytesRead + ", CurrentPacketLength = " + CurrentPacketLength);
                switch(CurrentState)
                {
                    case ReceiveStates.FIRSTHEADER:
                        if (ReadBuffer[x] == 0xBE)
                        {
			  //			  DebugLog("KSPSerialIO: received first header");
			  CurrentState = ReceiveStates.SECONDHEADER;
                        }
                        break;
                    case ReceiveStates.SECONDHEADER:
                        if (ReadBuffer[x] == 0xEF)
                        {
			  //			  DebugLog("KSPSerialIO: received second header");
			  CurrentState = ReceiveStates.SIZE;
                        }
			else
                        {
			  //			  DebugLog("KSPSerialIO: did not receive second header");
			  CurrentState = ReceiveStates.FIRSTHEADER;
                        }
                        break;
                    case ReceiveStates.SIZE:
                        CurrentPacketLength = ReadBuffer[x];
                        CurrentBytesRead = 0;
                        CurrentState = ReceiveStates.PAYLOAD;
			//			DebugLog("KSPSerialIO: received size " + CurrentPacketLength);
                        break;
                    case ReceiveStates.PAYLOAD:
                        PayloadBuffer[CurrentBytesRead] = ReadBuffer[x];
                        CurrentBytesRead++;
                        if (CurrentBytesRead >= CurrentPacketLength)
                        {
			  //			  DebugLog("KSPSerialIO: finished payload");
			  CurrentState = ReceiveStates.CS;
                        }
                        break;
                    case ReceiveStates.CS:
                        if (CompareChecksum(ReadBuffer[x]))
                        {
			  // DebugLog("KSPSerialIO: checksum OK, id = " + PayloadBuffer[0] + ", CurrentBytesRead = " + CurrentBytesRead);
			  SerialMutex.WaitOne();
			  Buffer.BlockCopy(PayloadBuffer, 0, NewPacketBuffer, 0, CurrentBytesRead);
			  SerialMutex.ReleaseMutex();
			  NewPacketFlag = true;
                        }
			//   		        DebugLog("KSPSerialIO: returning to first header");
                        CurrentState = ReceiveStates.FIRSTHEADER;
                        break;
                }
            }
	    //	    DebugLog("KSPSerialIO: ReceivedDataEvent end");
        }

        private static Boolean CompareChecksum(byte readCS)
        {
            byte calcCS = CurrentPacketLength;
            for (int i=0; i<CurrentPacketLength; i++)
            {
                calcCS ^= PayloadBuffer[i];
            }
	    bool checksumOK = calcCS == readCS;
	    //	    DebugLog("KSPSerialIO: checksumOK = " + checksumOK + ", calcCS = " + calcCS + ", readCS = " + readCS);
            return checksumOK;
        }

        //these are copied from the intarwebs, converts struct to byte array
        private static byte[] StructureToByteArray(object obj)
        {
            int len = Marshal.SizeOf(obj);
            byte[] arr = new byte[len];
            IntPtr ptr = Marshal.AllocHGlobal(len);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, len);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        private static object ByteArrayToStructure(byte[] bytearray, object obj)
        {
            int len = Marshal.SizeOf(obj);

            IntPtr i = Marshal.AllocHGlobal(len);

            Marshal.Copy(bytearray, 0, i, len);

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);

            return obj;
        }
        /*
          private static T ReadUsingMarshalUnsafe<T>(byte[] data) where T : struct
          {
          unsafe
          {
          fixed (byte* p = &data[0])
          {
          return (T)Marshal.PtrToStructure(new IntPtr(p), typeof(T));
          }
          }
          }
        */
        void initializeDataPackets()
        {
            VData = new VesselData();
            VData.id = VDid;

            CPacket = new ControlPacket();
            VControls.ControlGroup = new Boolean[11];
            LPacket.message = new char[200];

            LPacket = new LogPacket();

        }

        private static void HandShake()
        {
            DebugLog("KSPSerialIO: Handshake received - " + ReceivedHPacket.M1.ToString() + ReceivedHPacket.M2.ToString() + ReceivedHPacket.M3.ToString());
        }

        private static void VesselControls()
        {
	    byte receivedSeq = CPacket.seq;
	    DebugLog("KSPSerialIO: Got control packet seq " + receivedSeq);
	    if (VData.ackseq == receivedSeq) {
	    	 DebugLog("KSPSerialIO: Dup sequence " + receivedSeq + ", skipping");
		 return;
	    }	
	    VData.ackseq = receivedSeq;
            VControls.Gear = BitMathByte(CPacket.MainControls, 4);
            VControls.Stage = BitMathByte(CPacket.MainControls, 0);
            VControls.SAS = BitMathByte(CPacket.MainControls, 7);
            VControls.RCS = BitMathByte(CPacket.MainControls, 6);
            ControlReceived = true;
	if (true)		return;
//            DebugLog("KSPSerialIO: VesselControls() start");
            VControls.SAS = BitMathByte(CPacket.MainControls, 7);
            VControls.RCS = BitMathByte(CPacket.MainControls, 6);
            VControls.Lights = BitMathByte(CPacket.MainControls, 5);
            VControls.Gear = BitMathByte(CPacket.MainControls, 4);
            VControls.Brakes = BitMathByte(CPacket.MainControls, 3);
            VControls.Precision = BitMathByte(CPacket.MainControls, 2);
            VControls.Abort = BitMathByte(CPacket.MainControls, 1);
            VControls.Stage = BitMathByte(CPacket.MainControls, 0);
            VControls.Pitch = (float)CPacket.Pitch / 1000.0F;
            VControls.Roll = (float)CPacket.Roll / 1000.0F;
            VControls.Yaw = (float)CPacket.Yaw / 1000.0F;
            VControls.TX = (float)CPacket.TX / 1000.0F;
            VControls.TY = (float)CPacket.TY / 1000.0F;
            VControls.TZ = (float)CPacket.TZ / 1000.0F;
            VControls.WheelSteer = (float)CPacket.WheelSteer / 1000.0F;
            VControls.Throttle = (float)CPacket.Throttle / 1000.0F;
            VControls.WheelThrottle = (float)CPacket.WheelThrottle / 1000.0F;
            VControls.SASMode = (int)CPacket.NavballSASMode & 0x0F;
            VControls.SpeedMode = (int)(CPacket.NavballSASMode >> 4);

            for (int j = 1; j <= 10; j++)
            {
                VControls.ControlGroup[j] = BitMathUshort(CPacket.ControlGroup, j);
            }

//            DebugLog("KSPSerialIO: VesselControls() end");
        }

        private static Boolean BitMathByte(byte x, int n)
        {
            return ((x >> n) & 1) == 1;
        }

        private static Boolean BitMathUshort(ushort x, int n)
        {
            return ((x >> n) & 1) == 1;
        }

        private static void Unimplemented()
        {
            DebugLog("KSPSerialIO: Packet id unimplemented");
        }

        private static void debug()
        {
            KSPSerialPort.DebugLog("KSPSerialIO: " + Port.BytesToRead.ToString() + "BTR");
        }


        public static void ControlStatus(int n, Boolean s)
        {
            if (s)
                VData.ActionGroups |= (UInt16)(1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
            else
                VData.ActionGroups &= (UInt16)~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
        }

        public static void PortCleanup()
        {
	  if (isPortOpen(KSPSerialPort.Port))
          {
	    KSPSerialPort.Port.Close();
	    KSPSerialPort.Port.Dispose();
	    KSPSerialPort.Port = null;
   	    KSPSerialPort.abortThread();
	    DebugLog("KSPSerialIO: Port closed");
          }
        }
    }

    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPSerialIO : MonoBehaviour
    {
        private long lastUpdate = 0;
        private double deltaT = 1.0f;
        private double missionTime = 0;
        private double missionTimeOld = 0;

        private long refreshPeriod = 200;
        public static Vessel ActiveVessel = new Vessel();

        public Guid VesselIDOld;

        IOResource TempR = new IOResource();

//        private static Boolean wasSASOn = false;

        private ScreenMessageStyle KSPIOScreenStyle = ScreenMessageStyle.UPPER_RIGHT;

        void Awake()
        {
	  KSPSerialPort.DebugLog("KSPSerialIO: KSPSerial.Awake()");
            ScreenMessages.PostScreenMessage("IO awake", 10f, KSPIOScreenStyle);
            //refreshPeriod = SettingsNStuff.refreshPeriod;
        }

        void Update()
        {
	  //	  DebugLog("KSPSerialIO: KSPSerial.Update()");
	  if (FlightGlobals.ActiveVessel != null && KSPSerialPort.isPortOpen(KSPSerialPort.Port))
            {
                //DebugLog("KSPSerialIO: 1");
		manageActiveVessel();
		sendVData();
		processCommands();
            }
            else
            {
                //DebugLog("KSPSerialIO: ActiveVessel not found");
            }
        }
	
        void Start()
        {
            if (KSPSerialPort.DisplayFound)
            {
                Thread.Sleep(200);

                ActiveVessel.OnPostAutopilotUpdate -= AxisInput;
                ActiveVessel = FlightGlobals.ActiveVessel;
                ActiveVessel.OnPostAutopilotUpdate += AxisInput;
		// syncInputs();
                ScreenMessages.PostScreenMessage("KerbalController Start complete");
                KSPSerialPort.DebugLog("KSPSerialIO: KerbalController Start complete");
            }
            else
            {
                ScreenMessages.PostScreenMessage("KerbalController not found", 10f, KSPIOScreenStyle);
            }
        }

	void syncInputs()
	{
                //sync inputs at start
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.RCS, KSPSerialPort.VControls.RCS);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, KSPSerialPort.VControls.SAS);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Light, KSPSerialPort.VControls.Lights);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Gear, KSPSerialPort.VControls.Gear);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, KSPSerialPort.VControls.Brakes);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Abort, KSPSerialPort.VControls.Abort);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Stage, KSPSerialPort.VControls.Stage);

	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom01, KSPSerialPort.VControls.ControlGroup[1]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom02, KSPSerialPort.VControls.ControlGroup[2]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom03, KSPSerialPort.VControls.ControlGroup[3]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom04, KSPSerialPort.VControls.ControlGroup[4]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom05, KSPSerialPort.VControls.ControlGroup[5]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom06, KSPSerialPort.VControls.ControlGroup[6]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom07, KSPSerialPort.VControls.ControlGroup[7]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom08, KSPSerialPort.VControls.ControlGroup[8]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom09, KSPSerialPort.VControls.ControlGroup[9]);
	  ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom10, KSPSerialPort.VControls.ControlGroup[10]);
	}

	void manageActiveVessel()
	{
                //If the current active vessel is not what we were using, we need to remove controls from the old 
                //vessel and attache it to the current one
                if (ActiveVessel != null && ActiveVessel.id != FlightGlobals.ActiveVessel.id)
                {
                    ActiveVessel.OnPostAutopilotUpdate -= AxisInput;
                    ActiveVessel = FlightGlobals.ActiveVessel;
                    ActiveVessel.OnPostAutopilotUpdate += AxisInput;
                    //sync some inputs on vessel switch
                    ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.RCS, KSPSerialPort.VControls.RCS);
                    ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, KSPSerialPort.VControls.SAS);
                    KSPSerialPort.DebugLog("KSPSerialIO: ActiveVessel changed");
                }
                else
                {
                    ActiveVessel = FlightGlobals.ActiveVessel;
                }
	}

	void processCommands()
	{

                #region inputs
                if (KSPSerialPort.ControlReceived)
                {
                    /*
                    ScreenMessages.PostScreenMessage("Nav Mode " + KSPSerialPort.CPacket.NavballSASMode.ToString());
                    
                     ScreenMessages.PostScreenMessage("SAS: " + KSPSerialPort.VControls.SAS.ToString() +
                     ", RCS: " + KSPSerialPort.VControls.RCS.ToString() +
                     ", Lights: " + KSPSerialPort.VControls.Lights.ToString() +
                     ", Gear: " + KSPSerialPort.VControls.Gear.ToString() +
                     ", Brakes: " + KSPSerialPort.VControls.Brakes.ToString() +
                     ", Precision: " + KSPSerialPort.VControls.Precision.ToString() +
                     ", Abort: " + KSPSerialPort.VControls.Abort.ToString() +
                     ", Stage: " + KSPSerialPort.VControls.Stage.ToString(), 10f, KSPIOScreenStyle);
                    
                     DebugLog("KSPSerialIO: SAS: " + KSPSerialPort.VControls.SAS.ToString() +
                     ", RCS: " + KSPSerialPort.VControls.RCS.ToString() +
                     ", Lights: " + KSPSerialPort.VControls.Lights.ToString() +
                     ", Gear: " + KSPSerialPort.VControls.Gear.ToString() +
                     ", Brakes: " + KSPSerialPort.VControls.Brakes.ToString() +
                     ", Precision: " + KSPSerialPort.VControls.Precision.ToString() +
                     ", Abort: " + KSPSerialPort.VControls.Abort.ToString() +
                     ", Stage: " + KSPSerialPort.VControls.Stage.ToString());
                     */

                    //if (FlightInputHandler.RCSLock != KSPSerialPort.VControls.RCS)
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.RCS, KSPSerialPort.VControls.RCS);
                        //ScreenMessages.PostScreenMessage("RCS: " + KSPSerialPort.VControls.RCS.ToString(), 10f, KSPIOScreenStyle);


                    //if (ActiveVessel.ctrlState.killRot != KSPSerialPort.VControls.SAS)
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, KSPSerialPort.VControls.SAS);
                        //ScreenMessages.PostScreenMessage("SAS: " + KSPSerialPort.VControls.SAS.ToString(), 10f, KSPIOScreenStyle);

                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Light, KSPSerialPort.VControls.Lights);

                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Gear, KSPSerialPort.VControls.Gear);

                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, KSPSerialPort.VControls.Brakes);

                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Abort, KSPSerialPort.VControls.Abort);

			if (KSPSerialPort.VControls.Stage)
			{
			  KSPSerialPort.DebugLog("KSPSerialIO: ActivateNextStage");
			  StageManager.ActivateNextStage();
			}

                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Stage, KSPSerialPort.VControls.Stage);
			KSPSerialPort.VControls.Stage = false;

                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom01, KSPSerialPort.VControls.ControlGroup[1]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom02, KSPSerialPort.VControls.ControlGroup[2]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom03, KSPSerialPort.VControls.ControlGroup[3]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom04, KSPSerialPort.VControls.ControlGroup[4]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom05, KSPSerialPort.VControls.ControlGroup[5]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom06, KSPSerialPort.VControls.ControlGroup[6]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom07, KSPSerialPort.VControls.ControlGroup[7]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom08, KSPSerialPort.VControls.ControlGroup[8]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom09, KSPSerialPort.VControls.ControlGroup[9]);
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.Custom10, KSPSerialPort.VControls.ControlGroup[10]);

                        if (KSPSerialPort.VControls.SASMode != 0 && KSPSerialPort.VControls.SASMode < 11)
                        {
                            if (!ActiveVessel.Autopilot.CanSetMode((VesselAutopilot.AutopilotMode)(KSPSerialPort.VControls.SASMode - 1)))
                            {
                                ScreenMessages.PostScreenMessage("KSPSerialIO: SAS mode " + KSPSerialPort.VControls.SASMode.ToString() + " not avalible");
                            }
                            else
                            {
                                ActiveVessel.Autopilot.SetMode((VesselAutopilot.AutopilotMode)KSPSerialPort.VControls.SASMode - 1);
                            }
                        }

                        if (!((KSPSerialPort.VControls.SpeedMode == 0) || ((KSPSerialPort.VControls.SpeedMode == 3) && !TargetExists())))
                        {
                            FlightGlobals.SetSpeedMode((FlightGlobals.SpeedDisplayModes)(KSPSerialPort.VControls.SpeedMode - 1));
                        }


                    // temporarily disengage SAS while steering
                    if (Math.Abs(KSPSerialPort.VControls.Pitch) > SettingsNStuff.SASTol ||
                    Math.Abs(KSPSerialPort.VControls.Roll) > SettingsNStuff.SASTol ||
                    Math.Abs(KSPSerialPort.VControls.Yaw) > SettingsNStuff.SASTol)
                    {
                        ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                    }
                    else
                    {
                        if (KSPSerialPort.VControls.SAS == true)
                        {
                            ActiveVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
                        }
                    }

                    KSPSerialPort.ControlReceived = false;
                } //end ControlReceived
                #endregion

		}
		
	void sendVData()
	{
	  long nowTime = KSPSerialPort.stopwatch.ElapsedMilliseconds;
                if (nowTime - lastUpdate > refreshPeriod)
                {
		  //                    DebugLog("KSPSerialIO: sendVData");

                    lastUpdate = nowTime;

                    KSPSerialPort.ControlStatus((int)enumAG.SAS, ActiveVessel.ActionGroups[KSPActionGroup.SAS]);
                    KSPSerialPort.ControlStatus((int)enumAG.RCS, ActiveVessel.ActionGroups[KSPActionGroup.RCS]);
                    KSPSerialPort.ControlStatus((int)enumAG.Light, ActiveVessel.ActionGroups[KSPActionGroup.Light]);
                    KSPSerialPort.ControlStatus((int)enumAG.Gear, ActiveVessel.ActionGroups[KSPActionGroup.Gear]);
                    KSPSerialPort.ControlStatus((int)enumAG.Brakes, ActiveVessel.ActionGroups[KSPActionGroup.Brakes]);
                    KSPSerialPort.ControlStatus((int)enumAG.Abort, ActiveVessel.ActionGroups[KSPActionGroup.Abort]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom01, ActiveVessel.ActionGroups[KSPActionGroup.Custom01]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom02, ActiveVessel.ActionGroups[KSPActionGroup.Custom02]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom03, ActiveVessel.ActionGroups[KSPActionGroup.Custom03]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom04, ActiveVessel.ActionGroups[KSPActionGroup.Custom04]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom05, ActiveVessel.ActionGroups[KSPActionGroup.Custom05]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom06, ActiveVessel.ActionGroups[KSPActionGroup.Custom06]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom07, ActiveVessel.ActionGroups[KSPActionGroup.Custom07]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom08, ActiveVessel.ActionGroups[KSPActionGroup.Custom08]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom09, ActiveVessel.ActionGroups[KSPActionGroup.Custom09]);
                    KSPSerialPort.ControlStatus((int)enumAG.Custom10, ActiveVessel.ActionGroups[KSPActionGroup.Custom10]);

		    if (false)
		    {

		      List<Part> ActiveEngines = new List<Part>();
		      ActiveEngines = GetListOfActivatedEngines(ActiveVessel);

		      KSPSerialPort.VData.AP = (float)ActiveVessel.orbit.ApA;
		      KSPSerialPort.VData.PE = (float)ActiveVessel.orbit.PeA;
		      KSPSerialPort.VData.SemiMajorAxis = (float)ActiveVessel.orbit.semiMajorAxis;
		      KSPSerialPort.VData.SemiMinorAxis = (float)ActiveVessel.orbit.semiMinorAxis;
		      KSPSerialPort.VData.e = (float)ActiveVessel.orbit.eccentricity;
		      KSPSerialPort.VData.inc = (float)ActiveVessel.orbit.inclination;
		      KSPSerialPort.VData.VVI = (float)ActiveVessel.verticalSpeed;
		      KSPSerialPort.VData.G = (float)ActiveVessel.geeForce;
		      KSPSerialPort.VData.TAp = (int)Math.Round(ActiveVessel.orbit.timeToAp);
		      KSPSerialPort.VData.TPe = (int)Math.Round(ActiveVessel.orbit.timeToPe);
		      KSPSerialPort.VData.Density = (float)ActiveVessel.atmDensity;
		      KSPSerialPort.VData.TrueAnomaly = (float)ActiveVessel.orbit.trueAnomaly;
		      KSPSerialPort.VData.period = (int)Math.Round(ActiveVessel.orbit.period);

		      //DebugLog("KSPSerialIO: 3");
		      double ASL = ActiveVessel.mainBody.GetAltitude(ActiveVessel.CoM);
		      double AGL = (ASL - ActiveVessel.terrainAltitude);

		      if (AGL < ASL)
                        KSPSerialPort.VData.RAlt = (float)AGL;
		      else
                        KSPSerialPort.VData.RAlt = (float)ASL;

		      KSPSerialPort.VData.Alt = (float)ASL;
		      KSPSerialPort.VData.Vsurf = (float)ActiveVessel.srfSpeed;
		      KSPSerialPort.VData.Lat = (float)ActiveVessel.latitude;
		      KSPSerialPort.VData.Lon = (float)ActiveVessel.longitude;

		      TempR = GetResourceTotal(ActiveVessel, "LiquidFuel");
		      KSPSerialPort.VData.LiquidFuelTot = TempR.Max;
		      KSPSerialPort.VData.LiquidFuel = TempR.Current;

		      KSPSerialPort.VData.LiquidFuelTotS = (float)ProspectForResourceMax("LiquidFuel", ActiveEngines);
		      KSPSerialPort.VData.LiquidFuelS = (float)ProspectForResource("LiquidFuel", ActiveEngines);

		      TempR = GetResourceTotal(ActiveVessel, "Oxidizer");
		      KSPSerialPort.VData.OxidizerTot = TempR.Max;
		      KSPSerialPort.VData.Oxidizer = TempR.Current;

		      KSPSerialPort.VData.OxidizerTotS = (float)ProspectForResourceMax("Oxidizer", ActiveEngines);
		      KSPSerialPort.VData.OxidizerS = (float)ProspectForResource("Oxidizer", ActiveEngines);

		      TempR = GetResourceTotal(ActiveVessel, "ElectricCharge");
		      KSPSerialPort.VData.EChargeTot = TempR.Max;
		      KSPSerialPort.VData.ECharge = TempR.Current;
		      TempR = GetResourceTotal(ActiveVessel, "MonoPropellant");
		      KSPSerialPort.VData.MonoPropTot = TempR.Max;
		      KSPSerialPort.VData.MonoProp = TempR.Current;
		      TempR = GetResourceTotal(ActiveVessel, "IntakeAir");
		      KSPSerialPort.VData.IntakeAirTot = TempR.Max;
		      KSPSerialPort.VData.IntakeAir = TempR.Current;
		      TempR = GetResourceTotal(ActiveVessel, "SolidFuel");
		      KSPSerialPort.VData.SolidFuelTot = TempR.Max;
		      KSPSerialPort.VData.SolidFuel = TempR.Current;
		      TempR = GetResourceTotal(ActiveVessel, "XenonGas");
		      KSPSerialPort.VData.XenonGasTot = TempR.Max;
		      KSPSerialPort.VData.XenonGas = TempR.Current;

		      missionTime = ActiveVessel.missionTime;
		      deltaT = missionTime - missionTimeOld;
		      missionTimeOld = missionTime;

		      KSPSerialPort.VData.MissionTime = (UInt32)Math.Round(missionTime);
		      KSPSerialPort.VData.deltaTime = (float)deltaT;

		      KSPSerialPort.VData.VOrbit = (float)ActiveVessel.orbit.GetVel().magnitude;

		      //DebugLog("KSPSerialIO: 4");

		      KSPSerialPort.VData.MNTime = 0;
		      KSPSerialPort.VData.MNDeltaV = 0;

		      if (ActiveVessel.patchedConicSolver != null)
			{
			  if (ActiveVessel.patchedConicSolver.maneuverNodes != null)
			    {
			      if (ActiveVessel.patchedConicSolver.maneuverNodes.Count > 0)
				{
				  KSPSerialPort.VData.MNTime = (UInt32)Math.Round(ActiveVessel.patchedConicSolver.maneuverNodes[0].UT - Planetarium.GetUniversalTime());
				  //ScreenMessages.PostScreenMessage(KSPSerialPort.VData.MNTime.ToString());
				  //KSPSerialPort.VData.MNDeltaV = (float)ActiveVessel.patchedConicSolver.maneuverNodes[0].DeltaV.magnitude;
				  KSPSerialPort.VData.MNDeltaV = (float)ActiveVessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(ActiveVessel.patchedConicSolver.maneuverNodes[0].patch).magnitude; //Added JS
				}
			    }
			}

		      //DebugLog("KSPSerialIO: 5");

		      Quaternion attitude = updateHeadingPitchRollField(ActiveVessel);

		      KSPSerialPort.VData.Roll = (float)((attitude.eulerAngles.z > 180) ? (attitude.eulerAngles.z - 360.0) : attitude.eulerAngles.z);
		      KSPSerialPort.VData.Pitch = (float)((attitude.eulerAngles.x > 180) ? (360.0 - attitude.eulerAngles.x) : -attitude.eulerAngles.x);
		      KSPSerialPort.VData.Heading = (float)attitude.eulerAngles.y;


		      if (ActiveVessel.orbit.referenceBody != null)
			{
			  KSPSerialPort.VData.SOINumber = GetSOINumber(ActiveVessel.orbit.referenceBody.name);
			}

		      KSPSerialPort.VData.MaxOverHeat = GetMaxOverHeat(ActiveVessel);
		      KSPSerialPort.VData.MachNumber = (float)ActiveVessel.mach;
		      KSPSerialPort.VData.IAS = (float)ActiveVessel.indicatedAirSpeed;

		      KSPSerialPort.VData.CurrentStage = (byte)StageManager.CurrentStage;
		      KSPSerialPort.VData.TotalStage = (byte)StageManager.StageCount;

		      //target distance and velocity stuff                    

		      KSPSerialPort.VData.TargetDist = 0;
		      KSPSerialPort.VData.TargetV = 0;

		      if (TargetExists())
			{
			  KSPSerialPort.VData.TargetDist = (float)Vector3.Distance(FlightGlobals.fetch.VesselTarget.GetVessel().transform.position, ActiveVessel.transform.position);
			  KSPSerialPort.VData.TargetV = (float)FlightGlobals.ship_tgtVelocity.magnitude;
			}


		      KSPSerialPort.VData.NavballSASMode = (byte)(((int)FlightGlobals.speedDisplayMode + 1) << 4); //get navball speed display mode
		      if (ActiveVessel.ActionGroups[KSPActionGroup.SAS])
			{
			  KSPSerialPort.VData.NavballSASMode = (byte)(((int)FlightGlobals.ActiveVessel.Autopilot.Mode + 1) | KSPSerialPort.VData.NavballSASMode);
			}

#region debugjunk
		    }
                    /*
                      DebugLog("KSPSerialIO: Stage " + KSPSerialPort.VData.CurrentStage.ToString() + ' ' +
                      KSPSerialPort.VData.TotalStage.ToString()); 
                      DebugLog("KSPSerialIO: Overheat " + KSPSerialPort.VData.MaxOverHeat.ToString());
                      DebugLog("KSPSerialIO: Mach " + KSPSerialPort.VData.MachNumber.ToString());
                      DebugLog("KSPSerialIO: IAS " + KSPSerialPort.VData.IAS.ToString());
                    
                      DebugLog("KSPSerialIO: SOI " + ActiveVessel.orbit.referenceBody.name + KSPSerialPort.VData.SOINumber.ToString());
                    
                      ScreenMessages.PostScreenMessage(KSPSerialPort.VData.OxidizerS.ToString() + "/" + KSPSerialPort.VData.OxidizerTotS +
                      "   " + KSPSerialPort.VData.Oxidizer.ToString() + "/" + KSPSerialPort.VData.OxidizerTot);
                    */
                    //KSPSerialPort.VData.Roll = Mathf.Atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) * 180 / Mathf.PI;
                    //KSPSerialPort.VData.Pitch = Mathf.Atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) * 180 / Mathf.PI;
                    //KSPSerialPort.VData.Heading = Mathf.Asin(-2 * (x * z - w * y)) *180 / Mathf.PI;
                    //DebugLog("KSPSerialIO: Roll    " + KSPSerialPort.VData.Roll.ToString());
                    //DebugLog("KSPSerialIO: Pitch   " + KSPSerialPort.VData.Pitch.ToString());
                    //DebugLog("KSPSerialIO: Heading " + KSPSerialPort.VData.Heading.ToString());
                    //DebugLog("KSPSerialIO: VOrbit" + KSPSerialPort.VData.VOrbit.ToString());
                    //ScreenMessages.PostScreenMessage(ActiveVessel.ActionGroups[KSPActionGroup.RCS].ToString());
                    //DebugLog("KSPSerialIO: MNTime" + KSPSerialPort.VData.MNTime.ToString() + " MNDeltaV" + KSPSerialPort.VData.MNDeltaV.ToString());
                    //DebugLog("KSPSerialIO: Time" + KSPSerialPort.VData.MissionTime.ToString() + " Delta Time" + KSPSerialPort.VData.deltaTime.ToString());
                    //DebugLog("KSPSerialIO: Throttle = " + KSPSerialPort.CPacket.Throttle.ToString());
                    //ScreenMessages.PostScreenMessage(KSPSerialPort.VData.Fuelp.ToString());
                    //ScreenMessages.PostScreenMessage(KSPSerialPort.VData.RAlt.ToString());
                    //KSPSerialPort.Port.WriteLine("Success!");
                    /*
                    ScreenMessages.PostScreenMessage(KSPSerialPort.VData.LiquidFuelS.ToString() + "/" + KSPSerialPort.VData.LiquidFuelTotS +
                        "   " + KSPSerialPort.VData.LiquidFuel.ToString() + "/" + KSPSerialPort.VData.LiquidFuelTot);
                    
                    ScreenMessages.PostScreenMessage("MNTime " + KSPSerialPort.VData.MNTime.ToString() + " MNDeltaV " + KSPSerialPort.VData.MNDeltaV.ToString());
                    ScreenMessages.PostScreenMessage("TargetDist " + KSPSerialPort.VData.TargetDist.ToString() + " TargetV " + KSPSerialPort.VData.TargetV.ToString());
                     */
                    #endregion
		    KSPSerialPort.newVData = true;
                } //end refresh

	}


        #region utilities

        private Boolean TargetExists()
        {
            return (FlightGlobals.fetch.VesselTarget != null) && (FlightGlobals.fetch.VesselTarget.GetVessel() != null); //&& is short circuiting
        }

        private byte GetMaxOverHeat(Vessel V)
        {
            byte percent = 0;
            double sPercent = 0, iPercent = 0;
            double percentD = 0, percentP = 0;

            foreach (Part p in ActiveVessel.parts)
            {
                //internal temperature
                iPercent = p.temperature / p.maxTemp;
                //skin temperature
                sPercent = p.skinTemperature / p.skinMaxTemp;

                if (iPercent > sPercent)
                    percentP = iPercent;
                else
                    percentP = sPercent;

                if (percentD < percentP)
                    percentD = percentP;
            }

            percent = (byte)Math.Round(percentD * 100);
            return percent;
        }


        private IOResource GetResourceTotal(Vessel V, string resourceName)
        {
            IOResource R = new IOResource();

            foreach (Part p in V.parts)
            {
                foreach (PartResource pr in p.Resources)
                {
                    if (pr.resourceName.Equals(resourceName))
                    {
                        R.Current += (float)pr.amount;
                        R.Max += (float)pr.maxAmount;

                        break;
                    }
                }
            }

            if (R.Max == 0)
                R.Current = 0;

            return R;
        }

        private void AxisInput(FlightCtrlState s)
        {
            switch (SettingsNStuff.ThrottleEnable)
            {
                case 1:
                    s.mainThrottle = KSPSerialPort.VControls.Throttle;
                    break;
                case 2:
                    if (s.mainThrottle == 0)
                    {
                        s.mainThrottle = KSPSerialPort.VControls.Throttle;
                    }
                    break;
                case 3:
                    if (KSPSerialPort.VControls.Throttle != 0)
                    {
                        s.mainThrottle = KSPSerialPort.VControls.Throttle;
                    }
                    break;
                default:
                    break;

            }

            switch (SettingsNStuff.PitchEnable)
            {
                case 1:
                    s.pitch = KSPSerialPort.VControls.Pitch;
                    break;
                case 2:
                    if (s.pitch == 0)
                        s.pitch = KSPSerialPort.VControls.Pitch;
                    break;
                case 3:
                    if (KSPSerialPort.VControls.Pitch != 0)
                        s.pitch = KSPSerialPort.VControls.Pitch;
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.RollEnable)
            {
                case 1:
                    s.roll = KSPSerialPort.VControls.Roll;
                    break;
                case 2:
                    if (s.roll == 0)
                        s.roll = KSPSerialPort.VControls.Roll;
                    break;
                case 3:
                    if (KSPSerialPort.VControls.Roll != 0)
                        s.roll = KSPSerialPort.VControls.Roll;
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.YawEnable)
            {
                case 1:
                    s.yaw = KSPSerialPort.VControls.Yaw;
                    break;
                case 2:
                    if (s.yaw == 0)
                        s.yaw = KSPSerialPort.VControls.Yaw;
                    break;
                case 3:
                    if (KSPSerialPort.VControls.Yaw != 0)
                        s.yaw = KSPSerialPort.VControls.Yaw;
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.TXEnable)
            {
                case 1:
                    s.X = KSPSerialPort.VControls.TX;
                    break;
                case 2:
                    if (s.X == 0)
                        s.X = KSPSerialPort.VControls.TX;
                    break;
                case 3:
                    if (KSPSerialPort.VControls.TX != 0)
                        s.X = KSPSerialPort.VControls.TX;
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.TYEnable)
            {
                case 1:
                    s.Y = KSPSerialPort.VControls.TY;
                    break;
                case 2:
                    if (s.Y == 0)
                        s.Y = KSPSerialPort.VControls.TY;
                    break;
                case 3:
                    if (KSPSerialPort.VControls.TY != 0)
                        s.Y = KSPSerialPort.VControls.TY;
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.TZEnable)
            {
                case 1:
                    s.Z = KSPSerialPort.VControls.TZ;
                    break;
                case 2:
                    if (s.Z == 0)
                        s.Z = KSPSerialPort.VControls.TZ;
                    break;
                case 3:
                    if (KSPSerialPort.VControls.TZ != 0)
                        s.Z = KSPSerialPort.VControls.TZ;
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.WheelSteerEnable)
            {
                case 1:
                    s.wheelSteer = KSPSerialPort.VControls.WheelSteer;
                    break;
                case 2:
                    if (s.wheelSteer == 0)
                    {
                        s.wheelSteer = KSPSerialPort.VControls.WheelSteer;
                    }
                    break;
                case 3:
                    if (KSPSerialPort.VControls.WheelSteer != 0)
                    {
                        s.wheelSteer = KSPSerialPort.VControls.WheelSteer;
                    }
                    break;
                default:
                    break;
            }

            switch (SettingsNStuff.WheelThrottleEnable)
            {
                case 1:
                    s.wheelThrottle = KSPSerialPort.VControls.WheelThrottle;
                    break;
                case 2:
                    if (s.wheelThrottle == 0)
                    {
                        s.wheelThrottle = KSPSerialPort.VControls.WheelThrottle;
                    }
                    break;
                case 3:
                    if (KSPSerialPort.VControls.WheelThrottle != 0)
                    {
                        s.wheelThrottle = KSPSerialPort.VControls.WheelThrottle;
                    }
                    break;
                default:
                    break;
            }
        }

        private byte GetSOINumber(string name)
        {
            byte SOI;

            switch (name.ToLower())
            {
                case "sun":
                    SOI = 100;
                    break;
                case "moho":
                    SOI = 110;
                    break;
                case "eve":
                    SOI = 120;
                    break;
                case "gilly":
                    SOI = 121;
                    break;
                case "kerbin":
                    SOI = 130;
                    break;
                case "mun":
                    SOI = 131;
                    break;
                case "minmus":
                    SOI = 132;
                    break;
                case "duna":
                    SOI = 140;
                    break;
                case "ike":
                    SOI = 141;
                    break;
                case "dres":
                    SOI = 150;
                    break;
                case "jool":
                    SOI = 160;
                    break;
                case "laythe":
                    SOI = 161;
                    break;
                case "vall":
                    SOI = 162;
                    break;
                case "tylo":
                    SOI = 163;
                    break;
                case "bop":
                    SOI = 164;
                    break;
                case "pol":
                    SOI = 165;
                    break;
                case "eeloo":
                    SOI = 170;
                    break;
                default:
                    SOI = 0;
                    break;
            }
            return SOI;
        }

        // this recursive stage look up stuff stolen and modified from KOS and others
        public static List<Part> GetListOfActivatedEngines(Vessel vessel)
        {
            var retList = new List<Part>();

            foreach (var part in vessel.Parts)
            {
                foreach (PartModule module in part.Modules)
                {
                    var engineModule = module as ModuleEngines;
                    if (engineModule != null)
                    {
                        if (engineModule.getIgnitionState)
                        {
                            retList.Add(part);
                        }
                    }

                    var engineModuleFx = module as ModuleEnginesFX;
                    if (engineModuleFx != null)
                    {
                        if (engineModuleFx.getIgnitionState)
                        {
                            retList.Add(part);
                        }
                    }
                }
            }

            return retList;
        }

        public static double ProspectForResource(String resourceName, List<Part> engines)
        {
            List<Part> visited = new List<Part>();
            double total = 0;

            foreach (var part in engines)
            {
                total += ProspectForResource(resourceName, part, ref visited);
            }

            return total;
        }

        public static double ProspectForResource(String resourceName, Part engine)
        {
            List<Part> visited = new List<Part>();

            return ProspectForResource(resourceName, engine, ref visited);
        }

        public static double ProspectForResource(String resourceName, Part part, ref List<Part> visited)
        {
            double ret = 0;

            if (visited.Contains(part))
            {
                return 0;
            }

            visited.Add(part);

            foreach (PartResource resource in part.Resources)
            {
                if (resource.resourceName.ToLower() == resourceName.ToLower())
                {
                    ret += resource.amount;
                }
            }

            foreach (AttachNode attachNode in part.attachNodes)
            {
                if (attachNode.attachedPart != null //if there is a part attached here
                    && attachNode.nodeType == AttachNode.NodeType.Stack //and the attached part is stacked (rather than surface mounted)
                    && (attachNode.attachedPart.fuelCrossFeed //and the attached part allows fuel flow
                        )
                    && !(part.NoCrossFeedNodeKey.Length > 0 //and this part does not forbid fuel flow
                         && attachNode.id.Contains(part.NoCrossFeedNodeKey))) // through this particular node
                {


                    ret += ProspectForResource(resourceName, attachNode.attachedPart, ref visited);
                }
            }

            return ret;
        }

        public static double ProspectForResourceMax(String resourceName, List<Part> engines)
        {
            List<Part> visited = new List<Part>();
            double total = 0;

            foreach (var part in engines)
            {
                total += ProspectForResourceMax(resourceName, part, ref visited);
            }

            return total;
        }

        public static double ProspectForResourceMax(String resourceName, Part engine)
        {
            List<Part> visited = new List<Part>();

            return ProspectForResourceMax(resourceName, engine, ref visited);
        }

        public static double ProspectForResourceMax(String resourceName, Part part, ref List<Part> visited)
        {
            double ret = 0;

            if (visited.Contains(part))
            {
                return 0;
            }

            visited.Add(part);

            foreach (PartResource resource in part.Resources)
            {
                if (resource.resourceName.ToLower() == resourceName.ToLower())
                {
                    ret += resource.maxAmount;
                }
            }

            foreach (AttachNode attachNode in part.attachNodes)
            {
                if (attachNode.attachedPart != null //if there is a part attached here
                    && attachNode.nodeType == AttachNode.NodeType.Stack //and the attached part is stacked (rather than surface mounted)
                    && (attachNode.attachedPart.fuelCrossFeed //and the attached part allows fuel flow
                        )
                    && !(part.NoCrossFeedNodeKey.Length > 0 //and this part does not forbid fuel flow
                         && attachNode.id.Contains(part.NoCrossFeedNodeKey))) // through this particular node
                {


                    ret += ProspectForResourceMax(resourceName, attachNode.attachedPart, ref visited);
                }
            }

            return ret;
        }

        //Borrowed from MechJeb2
        private Quaternion updateHeadingPitchRollField(Vessel v)
        {
            Vector3d CoM, north, up;
            Quaternion rotationSurface;
            CoM = v.CoM;
            up = (CoM - v.mainBody.position).normalized;
            north = Vector3d.Exclude(up, (v.mainBody.position + v.mainBody.transform.up * (float)v.mainBody.Radius) - CoM).normalized;
            rotationSurface = Quaternion.LookRotation(north, up);
            return Quaternion.Inverse(Quaternion.Euler(90, 0, 0) * Quaternion.Inverse(v.GetTransform().rotation) * rotationSurface);
        }

        #endregion

        void FixedUpdate()
        {
        }

        void OnDestroy()
        {
            if (KSPSerialPort.Port.IsOpen)
            {
                KSPSerialPort.PortCleanup();
                ScreenMessages.PostScreenMessage("Port closed", 10f, KSPIOScreenStyle);
            }
        }
    }
}
