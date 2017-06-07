using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Net.Http;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Http;

namespace SignalRHost.Controllers
{
    [RoutePrefix("Api/Test")]
    public class TestController : ApiController
    {
        struct Status
        {
            public float ActualRoll, ActualPitch;
            public float MotorFR, MotorFL, MotorRR, MotorRL;
            public float Power, PowerPitch, PowerRoll;
            public float BatteryLimitationFactor;
        };

        struct Command
        {
            public float Roll, Pitch, Power;
            public bool DoSendStatusreport;
        };

        static object Lock = new object();
        static SerialPort port;
        static double i = 0;

        [HttpGet]
        [Route("/GetStatus")]
        public IHttpActionResult GetStatus()
        {
            lock (Lock)
            {
                if (port == null || port.IsOpen == false)
                {
                    port = new SerialPort("COM5");
                    port.BaudRate = 115200;
                    port.DtrEnable = true;
                    port.RtsEnable = true;
                    port.Open();
                }

                try
                {
                    port.Write("Q");
                    byte[] Buffer = new byte[Marshal.SizeOf<Status>()];
                    port.Read(Buffer, 0, Marshal.SizeOf<Status>());
                    Status currentStatus = fromBytes(Buffer);
                    Debug.WriteLine("Motor1: " + currentStatus.MotorFL + "\t" + "Motor2: " + currentStatus.MotorFR + "\t" + "Motor3: " + currentStatus.MotorRL + "\t" + "Motor4: " + currentStatus.MotorRR + "\t" + "Power: " + currentStatus.Power+ "\t" + "BatteryLimitationFactor: " + currentStatus.BatteryLimitationFactor);
                    return Json<Status>(currentStatus);
                }
                catch (Exception e)
                {
                    return BadRequest();
                }

            }
        }

        [HttpGet]
        [Route("/SetCommand")]
        public IHttpActionResult SetCommand(float power)
        {
            lock (Lock)
            {
                if (port == null || port.IsOpen == false)
                {
                    port = new SerialPort("COM5");
                    port.BaudRate = 115200;
                    port.DtrEnable = true;
                    port.RtsEnable = true;
                    port.Open();
                }

                try
                {
                    port.DiscardInBuffer();
                    port.Write("C");
                    byte[] Buffer = getBytes(new Command() { Power = power });
                    while (port.BytesToRead == 0)
                    {
                        ;
                    }
                    port.BaseStream.Write(Buffer, 0, Buffer.Length);
                    port.BaseStream.Flush();
                    return Ok();
                }
                catch (Exception e)
                {
                    return BadRequest();
                }

            }
        }
        static Status fromBytes(byte[] arr)
        {
            Status str = new Status();

            int size = Marshal.SizeOf(str);
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(arr, 0, ptr, size);

            str = (Status)Marshal.PtrToStructure(ptr, str.GetType());
            Marshal.FreeHGlobal(ptr);

            return str;
        }

        byte[] getBytes(Command cmd)
        {
            int size = Marshal.SizeOf(cmd);
            byte[] arr = new byte[size];

            IntPtr ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(cmd, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }
    }
}
