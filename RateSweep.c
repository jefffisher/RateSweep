#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "OrionPublicPacket.h"
#include "GeolocateTelemetry.h"
#include "linearalgebra.h"
#include "earthposition.h"
#include "mathutilities.h"
#include "OrionComm.h"

static void KillProcess(const char *pMessage, int Value);
static void ProcessArgs(int argc, char **argv);

static OrionPkt_t PktIn, PktOut;

// Pitch angle to maintain
static float PitchAngle = radiansf(-45.0f);

// Tracking gain for camera pitch
static float TrackingGain = 2.0f;

// Ground area width in radians
static float SweepWidth = radiansf(20.0f);

// Sweep rate in hertz
static float SweepRate = 0.25f;

int main(int argc, char **argv)
{
    OrionCmd_t Cmd;

    // Set up the rate command
    Cmd.Mode = ORION_MODE_RATE;
    Cmd.ImpulseTime = 0.5f;
    Cmd.Stabilized = TRUE;

    // Process the command line arguments
    ProcessArgs(argc, argv);

    // Now just loop forever, looking for packets
    while (1)
    {
        // Pull any pending packets off the comm port
        while (OrionCommReceive(&PktIn))
        {
            GeolocateTelemetry_t Geo;

            // If this is a valid geolocate telemetry packet
            if (DecodeGeolocateTelemetry(&PktIn, &Geo))
            {
                // Swept past limit, reverse direction
                if (((SweepWidth > 0) && (Geo.base.pan >= SweepWidth * 0.5f)) ||
                    ((SweepWidth < 0) && (Geo.base.pan <= SweepWidth * 0.5f)))
                    SweepWidth = -SweepWidth;

                // Tilt rate command is gain * pitch error
                Cmd.Target[GIMBAL_AXIS_TILT] = TrackingGain * subtractAnglesf(PitchAngle, Geo.cameraEuler[AXIS_PITCH]);

                // Pan rate command is just the width (in rad) * rate (in 1/s) == rad/s
                Cmd.Target[GIMBAL_AXIS_PAN] = SweepWidth * SweepRate;

                // Encode and send the command to the gimbal
                encodeOrionCmdPacketStructure(&PktOut, &Cmd);
                OrionCommSend(&PktOut);
            }
        }

        // Give the rest of the system 5ms to work
        fflush(stdout);
        usleep(5000);
    }

    // Finally, be done!
    return 0;
}

// This function just shuts things down consistently with a nice message for the user
static void KillProcess(const char *pMessage, int Value)
{
    // Print out the error message that got us here
    printf("%s\n", pMessage);
    fflush(stdout);

    // Close down the active file descriptors
    OrionCommClose();

    // Finally exit with the proper return value
    exit(Value);

}// KillProcess

static void ProcessArgs(int argc, char **argv)
{
    char Error[80];

    // If we can't connect to a gimbal, kill the app right now
    if (OrionCommOpen(&argc, &argv) == FALSE)
        KillProcess("", 1);

    // Use a switch with fall-through to overwrite the default geopoint
    switch (argc)
    {
    case 4: SweepRate = atof(argv[3]);
    case 3: SweepWidth = radiansf(atof(argv[2]));
    case 2: PitchAngle = radiansf(atof(argv[1]));
    case 1: break;                           // Serial port path
    // If there aren't enough arguments
    default:
        // Kill the application and print the usage info
        sprintf(Error, "USAGE: %s [/dev/ttyXXX] [pitch_angle_deg] [sweep_width_deg] [sweep_rate_hz]", argv[0]);
        KillProcess(Error, -1);
        break;
    };

}// ProcessArgs
