using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GMVD
{
    public class SensorData
    {
        public string fileName = "";
        public float t;
        public float stillness = 0;
        public Vector3 gyro;
        public Vector3 accel;
        public Vector3 magnet;
        public float alpha = 0;
        public Quaternion gmvd;
        public Quaternion kalman;
        //public SensorData(String[] line) {
        //    this.fileName = "";
        //    this.t = Convert.ToSingle(line[0]);
        //    this.stillness = Convert.ToSingle(line[1]);
        //    this.gyro = new Vector3(Convert.ToSingle(line[2]), Convert.ToSingle(line[3]), Convert.ToSingle(line[4]));
        //    this.accel = new Vector3(Convert.ToSingle(line[5]), Convert.ToSingle(line[6]), Convert.ToSingle(line[7]));
        //    this.magnet = new Vector3(Convert.ToSingle(line[8]), Convert.ToSingle(line[9]), Convert.ToSingle(line[10]));
        //    this.alpha = Convert.ToSingle(line[11]);
            
        //    this.gmvd = new Quaternion(Convert.ToSingle(line[12]), Convert.ToSingle(line[13])
        //        , Convert.ToSingle(line[14]), Convert.ToSingle(line[15]));

        //    this.kalman = new Quaternion(Convert.ToSingle(line[16]), Convert.ToSingle(line[17])
        //        , Convert.ToSingle(line[18]), Convert.ToSingle(line[19]));
        //}

        public SensorData(float stillness, Vector3 gyro, Vector3 accel, Vector3 magnet, Quaternion km)
        {
            this.fileName = "";
            this.t = DateTime.Now.Millisecond;
            this.stillness = stillness;
            this.gyro = gyro;
            this.accel = accel;
            this.magnet = magnet;
            this.alpha = 99;

            //this.gmvd = new Quaternion(Convert.ToSingle(line[12]), Convert.ToSingle(line[13])
            //    , Convert.ToSingle(line[14]), Convert.ToSingle(line[15]));

            this.kalman = km;
        }
    }
}
