#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include "vrpn_Analog.h"
#include "vrpn_Analog_Output.h"

#ifndef _WIN32
#include <strings.h>
#endif

#define POLL_INTERVAL       (2000000)	  // time to poll if no response in a while (usec)
vrpn_Analog_Remote	  *ana;
vrpn_Analog_Output_Remote *anaout;

int done = 0;

bool analog_0_set = false;
vrpn_float64  analog_0;

unsigned long	duration(struct timeval t1, struct timeval t2)
{
	return (t1.tv_usec - t2.tv_usec) +
	       1000000L * (t1.tv_sec - t2.tv_sec);
}



/*****************************************************************************
 *
   Callback handlers
 *
 *****************************************************************************/

void	VRPN_CALLBACK handle_analog (void *, const vrpn_ANALOGCB a)
{
	int i;
	printf("Analogs: ");
	for (i = 0; i < a.num_channel; i++) {
		printf("%4.2f ",a.channel[i]);
	}
	analog_0 = a.channel[0];
	analog_0_set = true;
	printf("\n");
}


/*****************************************************************************
 *
   init - initialize everything
 *
 *****************************************************************************/

void init (const char * devicename)
{
	fprintf(stderr, "Zaber's name is %s.\n", devicename);
	ana = new vrpn_Analog_Remote (devicename);
	anaout = new vrpn_Analog_Output_Remote(devicename);

	// Set up the callback handlers
	printf("Analog update: Analogs: [new values listed]\n");
	ana->register_change_handler(NULL, handle_analog);
}	/* init */


void handle_cntl_c (int) {
    done = 1;
}

void shutdown (void) {

  fprintf(stderr, "\nIn control-c handler.\n");

  if (ana) delete ana;
  if (anaout) delete anaout;
  exit(0);
}

int main (int argc, char * argv [])
{
  struct timeval  timestamp;
  vrpn_gettimeofday(&timestamp, NULL);

#ifdef hpux
  char default_station_name [100];
  strcpy(default_station_name, "Analog0@localhost");
#else
  char default_station_name [] = { "Analog0@localhost" };
#endif

  const char * station_name = default_station_name;

  if (argc < 2) {
    fprintf(stderr, "Usage:  %s  Device_name\n"
                    "  Device_name:  VRPN name of data source to contact\n"
                    "    example:  Analog0@localhost\n",
            argv[0]);
    exit(0);
  }

  // parse args

  station_name = argv[1];

  // initialize the PC/station
  init(station_name);

  // signal handler so logfiles get closed right
  signal(SIGINT, handle_cntl_c);

  /* 
   * main interactive loop
   */
  while ( ! done ) {
    // Once every two seconds, ask the first analog to move
    // 10000 steps bigger (if it is less than 10000) or
    // 10000 steps shorter (if it is more than 10000)
    struct timeval current_time;
    vrpn_gettimeofday(&current_time, NULL);
    if ( duration(current_time,timestamp) > POLL_INTERVAL) {
      if (analog_0_set) {
	double newval;
	if (analog_0 > 10000) {
	  newval = analog_0-10000;
	} else {
	  newval = analog_0+10000;
	}
	printf("Requesting change to %lf\n", newval);
	anaout->request_change_channel_value(0, newval);
      } else {
	printf("No value yet from Zaber, not sending change request\n");
      }
      vrpn_gettimeofday(&timestamp, NULL);
    }

    // Let the devices do their things
    anaout->mainloop();
    ana->mainloop();

    // Sleep for 1ms so we don't eat the CPU
    vrpn_SleepMsecs(1);
  }
  shutdown();
  return 0;

}   /* main */
