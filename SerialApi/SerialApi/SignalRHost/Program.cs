using Microsoft.AspNet.SignalR.Client;
using Microsoft.Owin.Hosting;
using Serilog;
using Serilog.Core;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SignalRHost
{
    class Program
    {
        static void Main(string[] args)
        {
            Log.Logger = new LoggerConfiguration()
           .CreateLogger();

            string baseAddress = "http://localhost:9123/";

            // Start OWIN host 
            using (WebApp.Start<Startup>(url: baseAddress))
            {

                Console.WriteLine($"Server is running on {baseAddress}");
                Console.WriteLine("Press <enter> to stop server");
                Console.ReadLine();
            }
        }
    }
}
