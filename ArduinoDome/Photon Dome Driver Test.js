//*** CHECK THIS ProgID ***
var X = new ActiveXObject("ASCOM.Photon Dome Driver.Dome");
WScript.Echo("This is " + X.Name + ")");
// You may want to uncomment this...
// X.Connected = true;
X.SetupDialog();