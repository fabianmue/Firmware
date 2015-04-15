#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <sys/types.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "config.h"
#include "ps_sensorboard.h"
#include "ps_topics_handler.h"
#include "ps_data_update.h"

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

//thread priority
#define DAEMON_PRIORITY SCHED_PRIORITY_MAX - 20 ///daemon priority




/*
 * TODO:
 * 	- specify timeout for read => if no data is received, skip the request and terminate
 */





/**
 * daemon management function.
 */
__EXPORT int parser_sensorboard_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int parser_sb_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int parser_sensorboard_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("parser_sensorboard",
					     SCHED_DEFAULT,
					     DAEMON_PRIORITY,
					     2000,
					     parser_sb_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int parser_sb_thread_main(int argc, char *argv[])
{

	warnx("[parser_sensorboard] starting\n");


	//**HANDLE TOPICS
	struct subscribtion_fd_s subs;   //File-Descriptors of subscribed topics
	struct structs_topics_s strs;    //Struct of Interested Topics

	th_subscribe(&subs,&strs);       //Subscribe to interested Topics
	th_advertise();                  //Advertise Topics


	warnx("[parser_sensorboard] After topics\n");


	//**POLL FOR CHANGES IN SUBSCRIBED TOPICS
	struct pollfd fds[] = {			 // Polling Management
	       { .fd = subs.path_planning,       .events = POLLIN }
	};

	int poll_return;				//Return Value of the polling.



	//**INIT FUNCTIONS HERE
	int COMport;
	sb_init(&COMport);

	warnx("[parser_sensorboard] COM PORT open!\n");


	//**SET THE THREAD-STATUS TO RUNNING
	thread_running = true;



	/**MAIN THREAD-LOOP
	 * This is the main Thread Loop. It loops until the Process is killed.*/
	while (!thread_should_exit) {

		//**POLL FOR CHANGES IN THE SUBSCRIBED TOPICS
		poll_return = poll(fds, (sizeof(fds) / sizeof(fds[0])), TIMEOUT_POLL);

		if(poll_return == 0) {
			//No Topic contains changed data <=> no new Data available
			warnx("No new Data available\n");
		} else {
			//New Data is available

			if(poll_return < 0) {
				//An error occured during polling
				warnx("POLL ERR %d, %d", poll_return, errno);
				continue;
			} else {
				//Everything is OK and new Data is available
	            if(fds[0].revents & POLLIN){
	            	//copy new Pathplanning data
	            	orb_copy(ORB_ID(path_planning), subs.path_planning, &(strs.path_planning));

	            	//Update the state of sensorboard communication module
	            	sb_update_state(strs.path_planning.heading);

	            	//printf("Received new Heading!\n (JW) ");
	            }
			}
		}


		//Check if we received data over the serial interface
		//sb_write(&COMport,0x4A);
		//sb_read(&COMport);

		//Do updates of the variables
		du_handler(&COMport);



	} //END OF MAIN LOOP

	warnx("[parser_sensorboard] exiting.\n");

	thread_running = false;

	return 0;
}
