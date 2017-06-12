// --------------------------------------------------------------------------------
//
// ASCOM Dome driver based around Arduino+Photon
//
// Description: ASCOM Dome driver based around Arduino Leonardo and Particle Photon
// Leonardo is used as shutter controller and for Azimuth measurement
// Photon is used to control dome rotation and support cloud functions
//
// Implements:	ASCOM Dome interface version: 1.0.0
// Author:		Manoj Koushik manoj.koushik@gmail.com
//
// Edit Log: Initial version
//
// Date			Who	Vers	Description
// -----------	---	-----	-------------------------------------------------------
// 30-04-2017	MK	1.0.0	Initial version, created from ASCOM driver template
// --------------------------------------------------------------------------------
//

#define Dome

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Runtime.InteropServices;

using ASCOM;
using ASCOM.Astrometry;
using ASCOM.Astrometry.AstroUtils;
using ASCOM.Utilities;
using ASCOM.DeviceInterface;
using System.Globalization;
using System.Collections;
using System.Threading;
using System.IO;
using System.IO.Ports;

namespace ASCOM.Arduino
{
    //
    // Your driver's DeviceID is ASCOM.Arduino.Dome
    //
    // The Guid attribute sets the CLSID for ASCOM.Arduino.Dome
    // The ClassInterface/None addribute prevents an empty interface called
    // _Arduino from being created and used as the [default] interface
    //

    /// ASCOM Dome Driver for Arduino+Photon.
    [Guid("bbe256ee-31a8-4556-8c4e-f38e6a010615")]
    [ClassInterface(ClassInterfaceType.None)]
    public class Dome : IDomeV2
    {
        internal static string driverID = "ASCOM.Arduino.Dome";
        private static string driverDescription = "Arduino+Photon Dome Driver for Explora-Dome";

        internal static string comPortProfileName = "COM Port"; // Constants used for Profile persistence
        internal static string comPortDefault = "COM1";
        internal static string traceStateProfileName = "Trace Level";
        internal static string traceStateDefault = "false";

        internal static string comPort; // Variables to hold the currrent device configuration

        private bool connectedState;

        private Util utilities;

        private AstroUtils astroUtilities;

        internal static TraceLogger tl;

        private double azimuth;
        ShutterState shutter;
        private bool parked = true;
        // wait for up to 5 seconds for a response
        private int updateFrequency = 5000;
        private SerialPort photon;
        public Dome()
        {
            tl = new TraceLogger("", "ArduinoDome");
            ReadProfile(); // Read device configuration from the ASCOM Profile store

            tl.LogMessage("Dome", "Starting initialisation");

            connectedState = false; // Initialise connected to false
            utilities = new Util(); //Initialise util object
            astroUtilities = new AstroUtils(); // Initialise astro utilities object
            //TODO: Implement your additional construction here

            tl.LogMessage("Dome", "Completed initialisation");
        }



        #region Common properties and methods.

        public void SetupDialog()
        {
            // consider only showing the setup dialog if not connected
            // or call a different dialog if connected
            if (IsConnected)
                System.Windows.Forms.MessageBox.Show("Already connected, just press OK");

            using (SetupDialogForm F = new SetupDialogForm())
            {
                var result = F.ShowDialog();
                if (result == System.Windows.Forms.DialogResult.OK)
                {
                    WriteProfile(); // Persist device configuration values to the ASCOM Profile store
                }
            }
        }


        public ArrayList SupportedActions
        {
            get
            {
                tl.LogMessage("SupportedActions Get", "Returning list...");
                ArrayList supportedActions = new ArrayList();

                supportedActions.Add("OS");
                supportedActions.Add("CS");
                supportedActions.Add("VS");
                supportedActions.Add("SS");
                supportedActions.Add("SLEWAZ");
                supportedActions.Add("SYNCAZ");
                supportedActions.Add("STOPSLEW");
                supportedActions.Add("SETP");
                supportedActions.Add("P");
                supportedActions.Add("AZ");
                supportedActions.Add("LS");
                supportedActions.Add("US");
                supportedActions.Add("T");

                return supportedActions;
            }
        }

        public string Action(string actionName, string actionParameters)
        {
            LogMessage("Action", "ActionName {0}, ActionParameters {1} called", actionName, actionParameters);

            string deviceType = actionName.Substring(0, actionName.IndexOf(':'));
            string action = actionName.Substring(actionName.IndexOf(':') + 1);

            if (!deviceType.Equals("DC", StringComparison.OrdinalIgnoreCase))
            {
                return "";
            }

            if (action.Equals("OS", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("OS", false);
            }
            else if (action.Equals("CS", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("CS", false);
            }
            else if (action.Equals("SS", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("SS", false);
            }
            else if (action.Equals("VS", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("VS", false);
            }
            else if (action.Equals("SLEWAZ", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("SLEWAZ:" + actionParameters, false);
            }
            else if (action.Equals("SYNCAZ", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("SYNCAZ:" + actionParameters, false);
            }
            else if (action.Equals("STOPAZ", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("STOPAZ", false);
            }
            else if (action.Equals("SETP", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("SETP:" + actionParameters, false);
            }
            else if (action.Equals("P", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("P", false);
            }
            else if (action.Equals("AZ", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("AZ", false);
            }
            else if (action.Equals("LS", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("LS", false);
            }
            else if (action.Equals("US", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("US", false);
            }
            else if (action.Equals("T", StringComparison.OrdinalIgnoreCase))
            {
                return CommandString("T", false);
            }
            else
            {
                return "";
            }
        }

        public void CommandBlind(string command, bool raw)
        {
            throw new ASCOM.MethodNotImplementedException("CommandBlind");
        }

        public bool CommandBool(string command, bool raw)
        {
            throw new ASCOM.MethodNotImplementedException("CommandBool");
        }

        public string CommandString(string command, bool raw)
        {
            CheckConnected("CommandString");
            photon.WriteLine("DC:" + command);

            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (timer.ElapsedMilliseconds <= updateFrequency)
            {
                try
                {
                    string response = photon.ReadLine();
                    if (!string.IsNullOrEmpty(response) && response.StartsWith("DCR:"))
                        return response.Substring(response.IndexOf(':') + 1);
                }

                catch (Exception) { }

                Thread.Sleep(100);
            }

            return "";
        }

        public void Dispose()
        {
            // Clean up the tracelogger and util objects
            tl.Enabled = false;
            tl.Dispose();
            tl = null;
            utilities.Dispose();
            utilities = null;
            astroUtilities.Dispose();
            astroUtilities = null;
        }

        public bool Connected
        {
            get
            {
                return IsConnected;
            }
            set
            {
                tl.LogMessage("Connected", "Set {0}", value);
                if (value == IsConnected)
                    return;

                if (value)
                {
                    LogMessage("Connected Set", "Connecting to port {0}", comPort);
                    try
                    {
                        photon = new SerialPort(comPort, 57600);
                        photon.Open();
                        // Now that we are connected make sure we can talk to the dome controller
                        photon.ReadTimeout = 1000;
                        connectedState = true;
                        string resp = CommandString("AZ", false);
                        if (string.IsNullOrEmpty(resp))
                        {
                            LogMessage("Connected to {0} but no response", comPort);
                            connectedState = false;
                            photon.Close();
                            photon.Dispose();
                            photon = null;
                            throw new NotConnectedException("Connected but no response");
                        }
                    }
                    catch (Exception e)
                    {
                        LogMessage("Could not connect to port {0}", comPort);
                        throw new ASCOM.NotConnectedException(e.Message + "\nCould not connect to port ", e);
                    }
                }
                else
                {
                    LogMessage("Connected Unset", "Disconnecting to port {0}", comPort);

                    connectedState = false;

                    if (photon != null)
                    {
                        photon.Close();
                        photon.Dispose();
                        photon = null;
                    }
                }
            }
        }

        public string Description
        {
            get
            {
                tl.LogMessage("Description Get", driverDescription);
                return driverDescription;
            }
        }

        public string DriverInfo
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverInfo = "A Photon+Arduino based dome controller for Explora-Dome. Version: " + String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverInfo Get", driverInfo);
                return driverInfo;
            }
        }

        public string DriverVersion
        {
            get
            {
                Version version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version;
                string driverVersion = String.Format(CultureInfo.InvariantCulture, "{0}.{1}", version.Major, version.Minor);
                tl.LogMessage("DriverVersion Get", driverVersion);
                return driverVersion;
            }
        }

        public short InterfaceVersion
        {
            // set by the driver wizard
            get
            {
                LogMessage("InterfaceVersion Get", "2");
                return Convert.ToInt16("2");
            }
        }

        public string Name
        {
            get
            {
                string name = "Arduino+Photon Dome Controller";
                tl.LogMessage("Name Get", name);
                return name;
            }
        }

        #endregion

        #region IDome Implementation

        public void AbortSlew()
        {
            CommandString("STOPSLEW", false);
            tl.LogMessage("AbortSlew", "Completed");
        }

        public double Altitude
        {
            get
            {
                tl.LogMessage("Altitude Get", "Not implemented");
                throw new ASCOM.PropertyNotImplementedException("Altitude", false);
            }
        }

        public bool AtHome
        {
            get
            {
                tl.LogMessage("AtHome Get", "No Home Position. Returning false...");
                return false;
            }
        }

        public bool AtPark
        {
            get
            {
                string resp = CommandString("GETP", false);
                if (resp.StartsWith("GETP"))
                {
                    parked = bool.Parse(resp.Substring(resp.IndexOf(':') + 1));
                    tl.LogMessage("AtPark Get", parked.ToString());
                }
                return parked;
            }
        }

        public double Azimuth
        {
            get
            {
                string resp = CommandString("AZ", false);
                if (resp.StartsWith("AZ"))
                {
                    azimuth = float.Parse(resp.Substring(resp.IndexOf(':') + 1));
                    tl.LogMessage("Azimuth Get", azimuth.ToString());
                }
                return azimuth;
            }
        }

        public bool CanFindHome
        {
            get
            {
                tl.LogMessage("CanFindHome Get", false.ToString());
                return false;
            }
        }

        public bool CanPark
        {
            get
            {
                tl.LogMessage("CanPark Get", true.ToString());
                return true;
            }
        }

        public bool CanSetAltitude
        {
            get
            {
                tl.LogMessage("CanSetAltitude Get", false.ToString());
                return false;
            }
        }

        public bool CanSetAzimuth
        {
            get
            {
                tl.LogMessage("CanSetAzimuth Get", true.ToString());
                return true;
            }
        }

        public bool CanSetPark
        {
            get
            {
                tl.LogMessage("CanSetPark Get", true.ToString());
                return true;
            }
        }

        public bool CanSetShutter
        {
            get
            {
                tl.LogMessage("CanSetShutter Get", true.ToString());
                return true;
            }
        }

        public bool CanSlave
        {
            get
            {
                tl.LogMessage("CanSlave Get", false.ToString());
                return false;
            }
        }

        public bool CanSyncAzimuth
        {
            get
            {
                tl.LogMessage("CanSyncAzimuth Get", true.ToString());
                return true;
            }
        }

        public void CloseShutter()
        {
            CommandString("CS", false);
            tl.LogMessage("CloseShutter", "Shutter is being closed");
        }

        public void FindHome()
        {
            tl.LogMessage("FindHome", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("FindHome");
        }

        public void OpenShutter()
        {
            CommandString("OS", false);
            tl.LogMessage("OpenShutter", "Shutter is being opened");
        }

        public void Park()
        {
            CommandString("P", false);
            tl.LogMessage("Park", "Parking...");
        }

        public void SetPark()
        {
            CommandString("SETP:" + azimuth.ToString("F2"), false);
            tl.LogMessage("SetPark", "Setting park position to azimuth " + azimuth.ToString("F2"));
        }

        public ShutterState ShutterStatus
        {
            get
            {
                string resp = CommandString("S", false);
                if (resp.StartsWith("S"))
                {
                    shutter = (ShutterState) int.Parse(resp.Substring(resp.IndexOf(':') + 1));
                    tl.LogMessage("ShutterStatus", shutter.ToString());
                }
                return shutter;
            }
        }

        public bool Slaved
        {
            get
            {
                tl.LogMessage("Slaved Get", false.ToString());
                return false;
            }
            set
            {
                tl.LogMessage("Slaved Set", "not implemented");
                throw new ASCOM.PropertyNotImplementedException("Slaved", true);
            }
        }

        public void SlewToAltitude(double Altitude)
        {
            tl.LogMessage("SlewToAltitude", "Not implemented");
            throw new ASCOM.MethodNotImplementedException("SlewToAltitude");
        }

        public void SlewToAzimuth(double Azimuth)
        {
            CommandString("SLEWAZ:" + Azimuth.ToString("F2"), false);
            tl.LogMessage("SlewToAzimuth", "Slewing from " + azimuth.ToString() + " to " + Azimuth.ToString("F2"));
        }

        public bool Slewing
        {
            get
            {
                bool slewing = bool.Parse(CommandString("GETSLEW", false));
                tl.LogMessage("Slewing Get", slewing.ToString());

                return slewing;
            }
        }

        public void SyncToAzimuth(double Azimuth)
        {
            CommandString("SAZ:" + Azimuth.ToString("F2"), false);
            tl.LogMessage("SyncToAzimuth", "syncing to azimuth to " + Azimuth.ToString("F2"));
        }

        #endregion

        #region Private properties and methods

        #region ASCOM Registration

        private static void RegUnregASCOM(bool bRegister)
        {
            using (var P = new ASCOM.Utilities.Profile())
            {
                P.DeviceType = "Dome";
                if (bRegister)
                {
                    P.Register(driverID, driverDescription);
                }
                else
                {
                    P.Unregister(driverID);
                }
            }
        }

        [ComRegisterFunction]
        public static void RegisterASCOM(Type t)
        {
            RegUnregASCOM(true);
        }

        [ComUnregisterFunction]
        public static void UnregisterASCOM(Type t)
        {
            RegUnregASCOM(false);
        }

        #endregion

        private bool IsConnected
        {
            get
            {
                // TODO check that the driver hardware connection exists and is connected to the hardware
                return connectedState;
            }
        }

        private void CheckConnected(string message)
        {
            if (!IsConnected)
            {
                throw new ASCOM.NotConnectedException(message);
            }
        }

        internal void ReadProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Dome";
                tl.Enabled = Convert.ToBoolean(driverProfile.GetValue(driverID, traceStateProfileName, string.Empty, traceStateDefault));
                comPort = driverProfile.GetValue(driverID, comPortProfileName, string.Empty, comPortDefault);
            }
        }

        internal void WriteProfile()
        {
            using (Profile driverProfile = new Profile())
            {
                driverProfile.DeviceType = "Dome";
                driverProfile.WriteValue(driverID, traceStateProfileName, tl.Enabled.ToString());
                driverProfile.WriteValue(driverID, comPortProfileName, comPort.ToString());
            }
        }

        internal static void LogMessage(string identifier, string message, params object[] args)
        {
            var msg = string.Format(message, args);
            tl.LogMessage(identifier, msg);
        }
        #endregion
    }
}