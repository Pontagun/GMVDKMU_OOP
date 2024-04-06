using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Security.Cryptography;
//using System.Numerics;

using System.Threading;
//using System.Runtime.Intrinsics.X86;
using static System.Net.Mime.MediaTypeNames;
using System.IO.Ports;
using System.Numerics;
namespace GMVD
{
    public class SerialConnecter
    {
        //private SerialPort _sp;
        //public SerialPort SerialPort { get { return _sp; } }
        public SerialPort sp;

        

        private string portNumer;

        public static byte[] stream_timing_bytes = new byte[15];
        public static byte[] start_stream_bytes = new byte[3];
        public static byte[] interval = BitConverter.GetBytes(70000);
        public static byte[] delay = BitConverter.GetBytes(0);
        public static byte[] duration = BitConverter.GetBytes(0xFFFFFFFF);
        public static byte[] stop_command = { 0xF8, 0x00, 0x56, 0x56 };

        public static float Stillness0;
        public static Vector3 Gyro0, Accelero0, Magneto0;
        public static Quaternion IMUQuat0 = new Quaternion(0, 0, 0, 1);

        public SerialConnecter(string portNumber)
        {
            this.portNumer = portNumber;
            Array.Reverse(interval);    // byte[]
            Array.Reverse(delay);       // byte[]
            Array.Reverse(duration);    // byte[]
        }

        public SerialPort SerialConnect()
        {

            this.sp = new SerialPort(this.portNumer, 115200, Parity.None, 8, StopBits.One);

            this.sp.ReadTimeout = 500;
            this.sp.WriteTimeout = 500;

            try
            {
                this.sp.Open();
            }
            catch (Exception e)
            {
                Console.WriteLine($"Port {this.portNumer} is opening due to {e}");
            }

            return this.sp;
        }

        public Boolean SetSensorTSSCommand()
        {
            this.sp.Write(MARGSensor.stream_slots_bytes, 0, MARGSensor.stream_slots_bytes.Length);

            return true;
        }

        public Boolean SetStreamingInterval()
        {
            stream_timing_bytes[0] = MARGSensor.TSS_START_BYTE;
            stream_timing_bytes[1] = MARGSensor.TSS_SET_STREAMING_TIMING;
            interval.CopyTo(stream_timing_bytes, 2);
            delay.CopyTo(stream_timing_bytes, 6);
            duration.CopyTo(stream_timing_bytes, 10);

            stream_timing_bytes[14] = (byte)((stream_timing_bytes[1] + stream_timing_bytes[2]
                + stream_timing_bytes[3] + stream_timing_bytes[4]
                + stream_timing_bytes[5] + stream_timing_bytes[6]
                + stream_timing_bytes[7] + stream_timing_bytes[8]
                + stream_timing_bytes[9] + stream_timing_bytes[10]
                + stream_timing_bytes[11] + stream_timing_bytes[12]
                + stream_timing_bytes[13]) % 256);

            this.sp.Write(stream_timing_bytes, 0, stream_timing_bytes.Length);

            return true;
        }

        public Boolean SetTareSensor()
        {

            this.sp.Write(MARGSensor.tare_bytes, 0, 3);

            return true;
        }

        public Boolean StartStreaming()
        {

            this.sp.Write(MARGSensor.start_stream_bytes, 0, MARGSensor.start_stream_bytes.Length);

            return true;
        }
    }
}
