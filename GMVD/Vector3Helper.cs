using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GMVD
{
    public static class Vector3Helper
    {
        public static (Vector3, Vector3, Vector3) GetAverage(List<SensorData> sensorDatas)
        {
            int BUFF_SIZE = 3;
            Vector3 gyroAvg = new Vector3();
            Vector3 accelAvg = new Vector3();
            Vector3 magnetAvg = new Vector3();

            gyroAvg.X = sensorDatas.Take(BUFF_SIZE).Select(d => d.gyro.X).Average();
            gyroAvg.Y = sensorDatas.Take(BUFF_SIZE).Select(d => d.gyro.Y).Average();
            gyroAvg.Z = sensorDatas.Take(BUFF_SIZE).Select(d => d.gyro.Z).Average();

            accelAvg.X = sensorDatas.Take(BUFF_SIZE).Select(d => d.accel.X).Average();
            accelAvg.Y = sensorDatas.Take(BUFF_SIZE).Select(d => d.accel.Y).Average();
            accelAvg.Z = sensorDatas.Take(BUFF_SIZE).Select(d => d.accel.Z).Average();

            magnetAvg.X = sensorDatas.Take(BUFF_SIZE).Select(d => d.magnet.X).Average();
            magnetAvg.Y = sensorDatas.Take(BUFF_SIZE).Select(d => d.magnet.Y).Average();
            magnetAvg.Z = sensorDatas.Take(BUFF_SIZE).Select(d => d.magnet.Z).Average();

            return (gyroAvg, accelAvg, magnetAvg);
        }

        public static float GetKMuAngle(Vector3 magnetInert, Vector3 M_int0v)
        {
            // Error by angles
            float coskmuang = Vector3.Dot(magnetInert, M_int0v) / (magnetInert.Length() * M_int0v.Length());
            float gammaKmuang = Convert.ToSingle(Math.Acos(coskmuang));

            float slopekmua = 3.0f;
            float km1 = 1 + (-1 * slopekmua) * gammaKmuang;
            float kmuang = (1 + km1 + Math.Abs(1 + km1)) / 4;


            return kmuang;
        }

        public static float GetKMuMagnitude(List<Vector3> mag30List, Vector3 magnetInert)
        {
            // Error by magnitudes
            Vector3 mag30 = new Vector3(mag30List.Take(30).Select(d => d.X).Average()
    , mag30List.Take(30).Select(d => d.Y).Average(), mag30List.Take(30).Select(d => d.Z).Average());

            float NMagnitudeMagInert = magnetInert.Length() / mag30.Length();

            float angchg = Vector3Helper.anginertchg(magnetInert, mag30);

            float Magpenalty = NMagnitudeMagInert * angchg;

            float kmmag1 = 1 - Magpenalty;
            float kmmag = (kmmag1 + Math.Abs(kmmag1)) / 2;

            return kmmag;
        }

        public static Vector3 qrotbak(Quaternion q, Vector3 vin)
        {
            Quaternion vin4 = new Quaternion(vin, 0);
            Quaternion vout4 = q * (vin4 * Quaternion.Conjugate(q));
            Vector3 vout = new Vector3(vout4.X, vout4.Y, vout4.Z);

            return vout;
        }


        public static float anginertchg(Vector3 magnetInert, Vector3 mag30)
        {
            // dot(a, b) = | a || b | cos(eta)
            Vector3 a = magnetInert;
            Vector3 b = mag30;

            float aDotBSum = Vector3.Dot(a, b); // Dot as (a, b)
            float abMagProd = a.Length() * b.Length();

            double angleInRad = Math.Acos(aDotBSum / abMagProd);

            return Convert.ToSingle(angleInRad); // return acos()
        }
    }
}
