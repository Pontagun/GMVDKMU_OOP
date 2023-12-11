using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
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
        private static List<float> kmmergeList = new List<float>();


        static List<string> line = new List<string>();
        static void Main(string[] args)
        {
            float muK = 1.0f;
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
            float alphaMTNLNS = 0.0f;
            float alpha0 = 1.0f;
            float thisMuX = 1.0f;
            float thisMuY = 1.0f;
            using (StreamReader sr = new StreamReader("rec010GMV1.txt"))
            {
                try
                {
                    while (!sr.EndOfStream)
                    {
                        sensorDatas.Insert(0, new SensorData(sr.ReadLine().Split(","))); // This works like stack.

                        // Realtime readings.
                        //gyro0 = new Vector3(sensorDatas.First().gyro.X, sensorDatas.First().gyro.Y, sensorDatas.First().gyro.Z);
                        //accel0 = new Vector3(sensorDatas.First().accel.X, sensorDatas.First().accel.Y, sensorDatas.First().accel.Z);
                        //magnet0 = new Vector3(sensorDatas.First().magnet.X, sensorDatas.First().magnet.Y, sensorDatas.First().magnet.Z);
                        //stillness0 = Convert.ToSingle(sensorDatas.First().stillness) / 3;

                        (gyroAvg, accelAvg, magnetAvg, stillnessAvg) = Vector3Helper.GetAverage(sensorDatas);

                        gyroAvgList.Insert(0, gyroAvg);
                        accelAvgList.Insert(0, accelAvg);
                        magnetAvgList.Insert(0, magnetAvg);

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

                        stillnessAvg /= 3; // For some reasons, file recorded stillness ~3.

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
                        if (sensorDatas.Count == 1)
                        {
                            samplingInterval = Convert.ToSingle(sensorDatas.First().t);
                        }
                        else
                        {
                            samplingInterval = Convert.ToSingle(sensorDatas[0].t - sensorDatas[1].t);
                        }

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

                        alpha0List.Insert(0, alpha0);

                        accelInert = Vector3Helper.qrotbak(qOut, accelAvg);
                        magnetInert = Vector3Helper.qrotbak(qOut, magnetAvg);

                        mag30List.Add(magnetInert);

                        float kmuang = Vector3Helper.GetKMuAngle(magnetInert, M_int0v);
                        float kmmag = Vector3Helper.GetKMuMagnitude(mag30List, magnetInert);

                        float kmmerge = (kmuang + kmmag) / 2; // mean([kmuang(i) kmmag(i)]);
                        kmmergeList.Insert(0, kmmerge);

                        muK = GetKMU(kmmergeList.Take(WIN_SIZE).Average()
                            , alpha0List.Take(WIN_SIZE).Average());

                        //--- GMV
                        //qOut = Quaternion.Slerp(qG0, qGA0, alpha0); 

                        //--- GMVD
                        //    This is how to calculate mu in GMVD.
                        //thisMuY = (muWeight * thisMuX) + (1.0f - muWeight) * thisMuY;
                        //thisMuX = Convert.ToSingle(Math.Pow(Vector3Helper.GetMuPara(magnetAvg, qGA0, M_int0), 2));
                        //qOut = Quaternion.Slerp((Quaternion.Slerp(qG0, qGM0, thisMuY)), (Quaternion.Slerp(qG0, qGA0, alpha0)), alpha0);
                        
                        //--- GMVD with MuK
                        qOut = Quaternion.Slerp((Quaternion.Slerp(qG0, qGM0, muK)), (Quaternion.Slerp(qG0, qGA0, alpha0)), alphaMTNLNS);
                        line.Add(qOut.X + ", " + qOut.Y + ", " + qOut.Z + ", " + qOut.W + ", " + alphaMTNLNS);
                        //line.Add(stillnessGyro + ", " + stillnessAccel + ", " + alphaMTNLNS + ", " + alpha0);
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                }
                finally
                {
                    sr.Close();
                    File.AppendAllLines(Path.Combine(docPath, DateTime.Now.ToFileTime() + ".csv"), line);
                }
            }
        }

        public static float GetKMU(float tempkm, float alphamin)
        {
            int aminslope = 4;
            float alphamin1 = (alphamin * aminslope) - aminslope + 1;
            float alphamin2 = (alphamin1 + Math.Abs(alphamin1)) / 2;

            return tempkm * alphamin2;
        }

        public static float GetSensorDiff(List<Vector3> sensorDatas) {

            Vector3 deltaVector;
            float deltaVectorAxisMax = 0.0f;
            const float thMTNLNS = 0.5f;
            float valTMTNLNS = 0;

            if (sensorDatas.Count > 5)
            {
                deltaVector = sensorDatas[5] - sensorDatas[0];
            }
            else {
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

            // Checked True line.Add(deltaVector.X + ", " + deltaVector.Y + ", " + deltaVector.Z + ", " + deltaVectorAxisMax + ", " + valTMTNLNS);

            return valTMTNLNS;
        }

        public static float GetGammaFilter(float stillnessSensor, float oldAlpha) {

            // These variables use name as Nann's paper at HCII2022.
            float alphaG = -10.0f;
            float Walpha = .5f;

            alphaG = (Walpha * stillnessSensor) + (1 - Walpha) * (oldAlpha);

            return alphaG;
        }

        public static float GetLinearEquation(float value, float slope) {

            float alphaPrime = (slope * value) + (1);
            float alpha = (alphaPrime + (Math.Abs(alphaPrime))) / 2;

            return value;
        }
    }
}