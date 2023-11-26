using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace GMVD
{
    internal static class QuaternionHelper
    {
        public static Quaternion GetQuaternionG(Quaternion qOutPrev, Quaternion w0, float samplingInterval)
        {
            Quaternion qDot = (qOutPrev * w0);
            qDot *= 0.5f;

            Quaternion qG = myQuatIntegrate(qDot, qOutPrev, samplingInterval);

            return Quaternion.Normalize(qG);
        }

        public static Quaternion myQuatIntegrate(Quaternion dq, Quaternion q, float dt)
        {

            Quaternion omega = dq * Quaternion.Conjugate(q);
            omega = new Quaternion(2.0f * omega.X, 2.0f * omega.Y, 2.0f * omega.Z, 2.0f * omega.W);
            omega = new Quaternion((omega.X * dt) / 2.0f, (omega.Y * dt) / 2.0f, (omega.Z * dt) / 2.0f, (omega.W * dt) / 2.0f);
            float omega_norm2 = Convert.ToSingle(Math.Sqrt(Math.Pow(omega.X, 2) + Math.Pow(omega.Y, 2) + Math.Pow(omega.Z, 2)));
            Quaternion exp = new Quaternion();
            if (omega_norm2 != 0)
            {
                exp.X = Convert.ToSingle(Math.Exp(omega.W) * (Math.Sin(omega_norm2) / omega_norm2) * omega.X);
                exp.Y = Convert.ToSingle(Math.Exp(omega.W) * (Math.Sin(omega_norm2) / omega_norm2) * omega.Y);
                exp.Z = Convert.ToSingle(Math.Exp(omega.W) * (Math.Sin(omega_norm2) / omega_norm2) * omega.Z);
                exp.W = Convert.ToSingle(Math.Exp(omega.W) * Math.Cos(omega_norm2));
            }
            else
            {
                exp.X = Convert.ToSingle(Math.Exp(omega.W) * omega.X);
                exp.Y = Convert.ToSingle(Math.Exp(omega.W) * omega.Y);
                exp.Z = Convert.ToSingle(Math.Exp(omega.W) * omega.Z);
                exp.W = Convert.ToSingle(Math.Exp(omega.W) * Math.Cos(omega_norm2));
            }

            Quaternion q_result = exp * q;

            return q_result;
        }

        public static Quaternion GetMagAngleDiff(Vector3 avgData, Vector3 calData)
        {

            Vector3 qAv0 = Vector3.Cross(avgData, calData);
            float qAw0 = (avgData.Length() * calData.Length()) + Vector3.Dot(avgData, calData);

            Quaternion deltaQa0 = Quaternion.Normalize(new Quaternion(qAv0, qAw0));
            //qGA0 = myQuatNormalize(qGA0 * deltaQa0);

            return deltaQa0;
        }
    }
}
