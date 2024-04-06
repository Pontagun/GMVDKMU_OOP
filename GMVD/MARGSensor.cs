using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GMVD
{
    public class MARGSensor
    {
        public static byte TSS_START_BYTE = 0xF7; // Start byte.
        public static byte TSS_SET_STREAMING_SLOTS = 0x50;
        public static byte TSS_SET_STREAMING_TIMING = 0x52;
        public static byte TSS_START_STREAMING = 0x55;
        public static byte TSS_STOP_STREAMING = 0x56;
        public static byte TSS_GET_SENSOR_MOTION = 0x2D;
        public static byte TSS_GET_RAD_PER_SEC_GYROSCOPE = 0x26;
        public static byte TSS_GET_CORRETED_LINEAR_ACC_AND_GRAVITY = 0x27;
        public static byte TSS_GET_CORRETED_COMPASS = 0x28;
        public static byte TSS_GET_TARED_ORIENTAITON_AS_QUAT = 0x00;
        public static byte TSS_NULL = 0xFF;
        public static byte TSS_TARE_CURRENT_ORIENTATION = 0x60;

        public static byte CHECK_SUM = (byte)((TSS_SET_STREAMING_SLOTS + TSS_GET_SENSOR_MOTION + TSS_GET_RAD_PER_SEC_GYROSCOPE + TSS_GET_CORRETED_LINEAR_ACC_AND_GRAVITY +
                                        TSS_GET_CORRETED_COMPASS + TSS_GET_TARED_ORIENTAITON_AS_QUAT + TSS_NULL + TSS_NULL + TSS_NULL) % 256);

        public static byte[] stream_slots_bytes = {TSS_START_BYTE,
                                        TSS_SET_STREAMING_SLOTS,
                                        TSS_GET_SENSOR_MOTION, // Slot0 - 4
                                        TSS_GET_RAD_PER_SEC_GYROSCOPE, // Slot1 - 12
                                        TSS_GET_CORRETED_LINEAR_ACC_AND_GRAVITY, // Slot2 - 12
                                        TSS_GET_CORRETED_COMPASS, // Slot3 - 12
                                        TSS_GET_TARED_ORIENTAITON_AS_QUAT, // Slot4 - 16
                                        TSS_NULL, // Slot5
                                        TSS_NULL, // Slot6
                                        TSS_NULL, // Slot7
                                        CHECK_SUM};

        public static byte[] stream_timing_bytes = new byte[15];
        public static byte[] start_stream_bytes = new byte[3];
        public static byte[] tare_bytes = { TSS_START_BYTE, TSS_TARE_CURRENT_ORIENTATION, TSS_TARE_CURRENT_ORIENTATION }; // <-- one command, checksum = command itself


        public static float bytesToFloat(byte[] raw_bytes, int offset)
        {
            byte[] big_bytes = new byte[4];
            big_bytes[0] = raw_bytes[offset + 3];
            big_bytes[1] = raw_bytes[offset + 2];
            big_bytes[2] = raw_bytes[offset + 1];
            big_bytes[3] = raw_bytes[offset + 0];
            return BitConverter.ToSingle(big_bytes, 0);
        }
    }

    public class SensorHelper
    {
        public static float computeAlpha(Queue<float> q, float prevAlpha)
        {

            float[] stillnessBuffer = q.ToArray();
            float w_g = 0.25f;
            int m_a = 1;
            float alpha_prime = 0.0f;
            float alpha_g = 0.0f;
            float alpha = 1.0f;

            if (q.Count > 2)
            {
                alpha_g = w_g * (float)(Math.Pow(stillnessBuffer[1], 2));
                alpha_g += (1 - w_g) * prevAlpha;
                alpha_prime = (m_a * alpha_g);

                alpha = (alpha_prime + Math.Abs(alpha_prime)) / 2;
            }

            return alpha;
        }

    }
}
