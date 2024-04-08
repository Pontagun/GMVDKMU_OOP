using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Numerics;
using static System.Net.Mime.MediaTypeNames;

namespace GMVD
{
    internal class Program
    {
        public const float GYRO_THRSHLDx = 0.0271f;
        public const float GYRO_THRSHLDy = 0.0232f;
        public const float GYRO_THRSHLDz = 0.0305f;
        public const float B_SCALING = 1.00f;
        public const float TM = 0.2f;
        public const float TA = 0.9f;
        public const float alpWeight = 0.25f;
        public const float muWeight = 0.25f;
        public const int WIN_SIZE = 8;
        public const int WIN_SIZ_ALPHA_ACCEL = 5;

        // These are realtime reading associate to GMVD.
        public static Vector3 gyro0;
        public static Vector3 accel0;
        public static Vector3 magnet0;

        // These are average values to check sensor's steady.
        public static Vector3 gyroAvg;
        public static Vector3 accelAvg;
        public static Vector3 magnetAvg;
        public static float stillnessAvg;

        private static int _trigcountX = 0;
        private static int _trigcountY = 0;
        private static int _trigcountZ = 0;

        static float prevAlphaX0 = 1.0f;
        static float prevAlphaY0 = 1.0f;
        static float thisAlphaX0 = 1.0f;
        static float thisAlphaY0 = 1.0f;
        static float prevMuX = 0.0f;
        static float prevMuY = 0.0f;

        public static Quaternion A_int0;
        public static Vector3 A_int0v;
        public static Quaternion M_int0;
        public static Vector3 M_int0v;

        private static List<SensorData> sensorDatas = new List<SensorData>();
        private static List<Vector3> gyroAvgList = new List<Vector3>();
        private static List<Vector3> accelAvgList = new List<Vector3>();
        private static List<Vector3> magnetAvgList = new List<Vector3>();
        private static List<Vector3> mag30List = new List<Vector3>();
        private static List<float> alpha0List = new List<float>();
        private static List<float> alphaMTNLNSList = new List<float>();
        private static List<float> kmmergeList = new List<float>();


        static List<string> line = new List<string>();

        static float Stillness0;
        static Vector3 Gyro0, Accelero0, Magneto0;
        static Quaternion IMUQuat0 = new Quaternion(0, 0, 0, 1);
        static byte[] read_bytes0 = new byte[56];

        private static SerialConnecter _serialPort;
        static void Main(string[] args)
        {
            float KM = 1.0f;
            
            //float alphaGMVD = 1.0f;
            //float alphaGMVDX = 1.0f;
            //float alphaGMVDY = 1.0f;

            long algoStart = 0;
            long algoEnd = 0;

            Vector3 bias = new Vector3(0.0f);
            Vector3 biasBuffer = new Vector3(0.0f);
            Vector3 unbiasedGyro = new Vector3(0.0f);

            Quaternion w0 = Quaternion.Identity;
            Quaternion q0 = Quaternion.Identity;

            Quaternion qG0 = Quaternion.Identity;

            Quaternion qG = Quaternion.Identity;
            Quaternion qGA0 = Quaternion.Identity;
            Quaternion dqGA0 = Quaternion.Identity;
            Quaternion qGM0 = Quaternion.Identity;
            Quaternion dqGM0 = Quaternion.Identity;

            Quaternion qOut = Quaternion.Identity;

            Vector3 accelInert;
            Vector3 magnetInert;

            Quaternion a40 = Quaternion.Identity;
            Vector3 a30 = new Vector3();
            Quaternion m40 = Quaternion.Identity;
            Vector3 m30 = new Vector3();

            float stillness0 = 0.0f;
            float samplingInterval = 0;

            string docPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            float stillnessGyro = 1f; // Gyro𝑀𝑇𝑁𝐿𝑁𝑆
            float stillnessAccel = 1f; // Accel𝑀𝑇𝑁𝐿𝑁𝑆
            float alphaMTNLNS = 1.0f;
            float alpha0 = 1.0f;
            float thisMuX = 1.0f;
            float thisMuY = 1.0f;

            byte[] interval = BitConverter.GetBytes(20000);
            byte[] delay = BitConverter.GetBytes(0);
            byte[] duration = BitConverter.GetBytes(0xFFFFFFFF);
            //_serialPort = new SerialConnecter("COM6");
            SerialPort _serialPort = new SerialPort("COM6", 115200, Parity.None, 8, StopBits.One);

            _serialPort.ReadTimeout = 500;
            _serialPort.WriteTimeout = 500;

            try
            {
                _serialPort.Open();
            }
            catch (Exception e)
            {
                Console.WriteLine($"Port {"Com6"} is opening due to {e}");
            }

            _serialPort.Write(MARGSensor.stream_slots_bytes, 0, MARGSensor.stream_slots_bytes.Length);

            // The data must be flipped to big endian before sending to sensor
            Array.Reverse(interval);    // byte[]
            Array.Reverse(delay);       // byte[]
            Array.Reverse(duration);    // byte[]
            byte[] stream_timing_bytes = new byte[15];
            byte[] start_stream_bytes = new byte[3];
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

            _serialPort.Write(stream_timing_bytes, 0, stream_timing_bytes.Length);
            
            _serialPort.Write(MARGSensor.tare_bytes, 0, 3);

            start_stream_bytes[0] = MARGSensor.TSS_START_BYTE;
            start_stream_bytes[1] = MARGSensor.TSS_START_STREAMING;
            start_stream_bytes[2] = MARGSensor.TSS_START_STREAMING;

            _serialPort.Write(start_stream_bytes, 0, start_stream_bytes.Length);

            //using (StreamReader sr = new StreamReader("rec010GMV1.txt"))
            //{

            try
            {
                while (!(Console.KeyAvailable && Console.ReadKey().Key == ConsoleKey.Escape))
                {
                    read_bytes0 = Enumerable.Repeat((byte)1, 59).ToArray();

                    for (int i = 0; i < 56; i++)
                    {
                        _serialPort.Read(read_bytes0, i, 1);
                    }

                    Stillness0 = MARGSensor.bytesToFloat(read_bytes0, 0);
                    Gyro0.X = MARGSensor.bytesToFloat(read_bytes0, 4);
                    Gyro0.Y = MARGSensor.bytesToFloat(read_bytes0, 8);
                    Gyro0.Z = MARGSensor.bytesToFloat(read_bytes0, 12);
                    Accelero0.X = MARGSensor.bytesToFloat(read_bytes0, 16);
                    Accelero0.Y = MARGSensor.bytesToFloat(read_bytes0, 20);
                    Accelero0.Z = MARGSensor.bytesToFloat(read_bytes0, 24);
                    Magneto0.X = MARGSensor.bytesToFloat(read_bytes0, 28);
                    Magneto0.Y = MARGSensor.bytesToFloat(read_bytes0, 32);
                    Magneto0.Z = MARGSensor.bytesToFloat(read_bytes0, 36);
                    IMUQuat0.X = MARGSensor.bytesToFloat(read_bytes0, 40);
                    IMUQuat0.Y = MARGSensor.bytesToFloat(read_bytes0, 44);
                    IMUQuat0.Z = MARGSensor.bytesToFloat(read_bytes0, 48);
                    IMUQuat0.W = MARGSensor.bytesToFloat(read_bytes0, 52);


                    //while (!sr.EndOfStream)
                    //{
                    sensorDatas.Insert(0, new SensorData(Stillness0, Gyro0, Accelero0, Magneto0, IMUQuat0)); // This works like stack.

                    algoStart = DateTime.Now.Ticks;

                    // Realtime readings.
                    //gyro0 = new Vector3(sensorDatas.First().gyro.X, sensorDatas.First().gyro.Y, sensorDatas.First().gyro.Z);
                    //accel0 = new Vector3(sensorDatas.First().accel.X, sensorDatas.First().accel.Y, sensorDatas.First().accel.Z);
                    //magnet0 = new Vector3(sensorDatas.First().magnet.X, sensorDatas.First().magnet.Y, sensorDatas.First().magnet.Z);
                    //stillness0 = Convert.ToSingle(sensorDatas.First().stillness) / 3;

                    (gyroAvg, accelAvg, magnetAvg, stillnessAvg) = Vector3Helper.GetAverage(sensorDatas);
                    //stillnessAvg /= 3; // For some reasons, file recorded stillness ~3.

                    gyroAvgList.Insert(0, gyroAvg);
                    accelAvgList.Insert(0, accelAvg);
                    magnetAvgList.Insert(0, magnetAvg);


                    //alphaGMVDY = alpWeight * (alphaGMVDX) + (1.0f - alpWeight) * (alphaGMVDY);
                    //alphaGMVDX = Convert.ToSingle(Math.Pow(stillnessAvg, 2));

                    ////alphaGMVD = alphaGMVDY;
                    //int ma = 1;
                    //alphaGMVD = (ma * alphaGMVDY) + (1 - ma);
                    //alphaGMVD = (alphaGMVD + Math.Abs(alphaGMVD)) / 2;

                    //alphaGMVD = alphaGMVD < 0.01f ? 0.5f: alphaGMVD;

                    // Correction checked.
                    stillnessGyro = GetSensorDiff(gyroAvgList); // Gyro𝑀𝑇𝑁𝐿𝑁𝑆
                    stillnessAccel = GetSensorDiff(accelAvgList); // Accel𝑀𝑇𝑁𝐿𝑁𝑆

                    float smoothenStillnessGyro = GetGammaFilter(Convert.ToSingle(Math.Pow(stillnessGyro, 2)), alphaMTNLNS);
                    float smoothenStillnessAccel = GetGammaFilter(Convert.ToSingle(Math.Pow(stillnessAccel, 2)), alphaMTNLNS);
                    //line.Add(Math.Pow(stillnessGyro, 2) + ", " + smoothenStillnessGyro + ", " + Math.Pow(stillnessAccel, 2) + ", " 
                    //    + smoothenStillnessAccel + ", " + alphaMTNLNS + ", " + alpha0);

                    //float gyroMTNLNS = GetLinearEquation(smoothenStillnessGyro, 2f);
                    //float accelMTNLNS = GetLinearEquation(smoothenStillnessAccel, 2f);
                    //alphaMTNLNS = (gyroMTNLNS * gyroMTNLNS);
                    
                    alphaMTNLNS = smoothenStillnessAccel;


                    if (sensorDatas.Count == 1)
                    {
                        A_int0v = new Vector3(accelAvg.X, accelAvg.Y, accelAvg.Z);
                        A_int0 = new Quaternion(accelAvg.X, accelAvg.Y, accelAvg.Z, 0.0f);
                        M_int0v = new Vector3(magnetAvg.X, magnetAvg.Y, magnetAvg.Z);
                        M_int0 = new Quaternion(magnetAvg.X, magnetAvg.Y, magnetAvg.Z, 0.0f);
                    }

                    if (Math.Abs(gyroAvg.X) < GYRO_THRSHLDx)
                    {
                        _trigcountX += 1;
                        biasBuffer.X += gyroAvg.X;
                    }
                    else
                    {
                        _trigcountX = 0;
                        biasBuffer.X = 0;
                    }

                    if (_trigcountX == 5)
                    {
                        bias.X = biasBuffer.X / 5;
                        _trigcountX = 0;
                        biasBuffer.X = 0.0f;
                    }


                    if (Math.Abs(gyroAvg.Y) < GYRO_THRSHLDy)
                    {
                        _trigcountY += 1;
                        biasBuffer.Y += gyroAvg.Y;
                    }
                    else
                    {
                        _trigcountY = 0;
                        biasBuffer.Y = 0;
                    }

                    if (_trigcountY == 5)
                    {
                        bias.Y = biasBuffer.Y / 5;
                        _trigcountY = 0;
                        biasBuffer.Y = 0.0f;
                    }


                    if (Math.Abs(gyroAvg.Z) < GYRO_THRSHLDz)
                    {
                        _trigcountZ += 1;
                        biasBuffer.Z += gyroAvg.Z;
                    }
                    else
                    {
                        _trigcountZ = 0;
                        biasBuffer.Z = 0;
                    }

                    if (_trigcountZ == 5)
                    {
                        bias.Y = biasBuffer.Z / 5;
                        _trigcountZ = 0;
                        biasBuffer.Z = 0.0f;
                    }

                    // Removing Gyroscope Bias
                    unbiasedGyro = gyroAvg - (B_SCALING * bias);

                    w0 = new Quaternion(unbiasedGyro, 0.0f);

                    // TODO : sampling from sensor instead.

                    qG0 = qOut;
                    qG0 = QuaternionHelper.GetQuaternionG(qG0, w0, samplingInterval);

                    // Compute Gravity and Magnetic North Vectors
                    // TODO : Change all quaternion multiolication to Quaternion.Multipy.
                    // Suffix __'0' is a reading relating to "sensor reads".
                    a40 = Quaternion.Conjugate(qG0) * (A_int0 * qG0);
                    a30 = new Vector3(a40.X, a40.Y, a40.Z);
                    m40 = Quaternion.Conjugate(qG0) * (M_int0 * qG0);
                    m30 = new Vector3(m40.X, m40.Y, m40.Z);

                    dqGA0 = QuaternionHelper.GetMagAngleDiff(accelAvg, a30);
                    qGA0 = Quaternion.Normalize(qG0 * dqGA0);

                    dqGM0 = QuaternionHelper.GetMagAngleDiff(magnetAvg, m30);
                    qGM0 = Quaternion.Normalize(qG0 * dqGM0);

                    thisAlphaY0 = alpWeight * (thisAlphaX0) + (1.0f - alpWeight) * (thisAlphaY0);
                    thisAlphaX0 = Convert.ToSingle(Math.Pow(stillnessAvg, 2.0f));
                    alpha0 = Vector3Helper.GetAlphaPara(thisAlphaY0);

                    alphaMTNLNSList.Insert(0, alphaMTNLNS);
                    alpha0List.Insert(0, alpha0);

                    accelInert = Vector3Helper.qrotbak(qOut, accelAvg);
                    magnetInert = Vector3Helper.qrotbak(qOut, magnetAvg);

                    mag30List.Add(magnetInert);

                    float kmuang = Vector3Helper.GetKMuAngle(magnetInert, M_int0v);
                    float kmmag = Vector3Helper.GetKMuMagnitude(mag30List, magnetInert);

                    float kmmerge = (kmuang + kmmag) / 2; // mean([kmuang(i) kmmag(i)]);
                    kmmergeList.Insert(0, kmmerge);

                    KM = GetKMU(kmmergeList.Take(WIN_SIZE).Average()
                        , alphaMTNLNSList.Take(WIN_SIZE).Average());

                    //--- GMV
                    //qOut = Quaternion.Slerp(qG0, qGA0, alpha0); 

                    //--- GMVD
                    //    This is how to calculate mu in GMVD.
                    //thisMuY = (muWeight * thisMuX) + (1.0f - muWeight) * thisMuY;
                    //thisMuX = Convert.ToSingle(Math.Pow(Vector3Helper.GetMuPara(magnetAvg, qGA0, M_int0), 2));
                    //qOut = Quaternion.Slerp((Quaternion.Slerp(qG0, qGM0, thisMuY)), (Quaternion.Slerp(qG0, qGA0, alpha0)), alpha0);

                    //--- GMVD with MuK
                    qOut = Quaternion.Slerp((Quaternion.Slerp(qG0, qGM0, KM)), (Quaternion.Slerp(qG0, qGA0, alphaMTNLNS)), alphaMTNLNS);
                    algoEnd = DateTime.Now.Ticks;

                    TimeSpan elapsedSpan = new TimeSpan((algoEnd - algoStart));
                    line.Add(DateTime.Now.Millisecond + "," + Stillness0
                        + "," + alpha0 + "," + gyroAvg.X + "," + gyroAvg.Y + "," + gyroAvg.Z
                        + "," + accelAvg.X + "," + accelAvg.Y + "," + accelAvg.Z
                        + "," + magnetAvg.X + "," + magnetAvg.Y + "," + magnetAvg.Z
                        + "," + IMUQuat0.X + "," + IMUQuat0.Y + "," + IMUQuat0.Z + "," + IMUQuat0.W
                        + "," + qOut.X + "," + qOut.Y + "," + qOut.Z + "," + qOut.W + "," + alphaMTNLNS 
                        + "," + algoStart + "," + algoEnd + "," + (algoEnd - algoStart));
                    if(line.Count % 180000 == 0) {
                        File.AppendAllLines(Path.Combine(docPath, DateTime.Now.ToFileTime() + ".csv"), line);
                        line.Clear();
                    }
                   
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
            finally
            {
                //sr.Close();
                File.AppendAllLines(Path.Combine(docPath, DateTime.Now.ToFileTime() + ".csv"), line);
            }
            //}
        }

        public static float GetKMU(float tempkm, float alphamin)
        {
            int aminslope = 4;
            float alphamin1 = (alphamin * aminslope) - aminslope + 1;
            float alphamin2 = (alphamin1 + Math.Abs(alphamin1)) / 2;

            return tempkm * alphamin2;
        }

        public static float GetSensorDiff(List<Vector3> sensorDatas)
        {

            Vector3 deltaVector;
            float deltaVectorAxisMax = 0.0f;
            const float thMTNLNS = 0.5f;
            float valTMTNLNS = 0;

            if (sensorDatas.Count > WIN_SIZ_ALPHA_ACCEL)
            {
                deltaVector = sensorDatas[WIN_SIZ_ALPHA_ACCEL] - sensorDatas[0];
            }
            else
            {
                deltaVector = sensorDatas[0];
            }

            deltaVectorAxisMax = Math.Max(Math.Abs(deltaVector.X), Math.Max(Math.Abs(deltaVector.Y), Math.Abs(deltaVector.Z)));

            if (deltaVectorAxisMax <= thMTNLNS)
            {
                valTMTNLNS = 1 - (deltaVectorAxisMax / thMTNLNS);
            }
            else
            {
                valTMTNLNS = 0;
            }

            return valTMTNLNS;
        }

        public static float GetGammaFilter(float stillnessSensor, float oldAlpha)
        {

            // These variables use name as Nann's paper at HCII2022.
            float alphaG = -10.0f;
            float Walpha = .5f;

            alphaG = (Walpha * stillnessSensor) + (1 - Walpha) * (oldAlpha);

            return alphaG;
        }

        public static float GetLinearEquation(float value, float slope)
        {

            float alphaPrime = (slope * value) + (1);
            float alpha = (alphaPrime + (Math.Abs(alphaPrime))) / 2;

            return value;
        }
    }
}