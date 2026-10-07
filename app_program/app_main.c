// #include "vehicle_controller.h"
// #include "motion_control.h"
// #include "ir_sensor.h"
// #include "line_following.h"
// #include "barcode.h"
// #include "obstacle.h"
// #include "imu.h"
// #include "wifi_app.h"

// void usermain(void)
// {
//     ir_sensor_init();
//     motion_control_init();
//     line_following_init();
//     barcode_init();
//     obstacle_init();
//     imu_init();
//     wifi_app_init();
//     vehicle_controller_init();

//     /* Create/start RTOS tasks */
// }
/*
 * Robot hardware bring-up baseline
 *
 * Purpose:
 * Before the five buddies begin implementing their individual project
 * algorithms, this program verifies that the common robot hardware works
 * reliably under micro T-Kernel.
 *
 * Hardware tested here:
 *
 * Buddy 2:
 *   - Left/right motor driver initialisation
 *   - Left/right wheel encoders
 *
 * Buddy 3:
 *   - Left/centre/right IR sensors
 *
 * Buddy 4:
 *   - GY-511 LSM303DLHC accelerometer
 *
 * Buddy 5:
 *   - Ultrasonic distance sensor
 *   - Servo initialisation
 *
 * Motors are kept STOPPED unless ROBOT_RUN_MOTOR_SELF_TEST is enabled
 * in app_config.h.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "usb_console_compat.h"
#include "demo_tasks.h"
<<<<<<< Updated upstream
=======

#include "app_config.h"
#include "imu.h"
#include "line_following.h"
#include "obstacle.h"
#include "motion_control.h"
>>>>>>> Stashed changes


/*
 * Stack size for each robot test task.
 */
#define STACK_SZ 4096


/*
 * Set TRUE after motion_control has successfully initialised.
 *
 * The optional motor test waits for this before attempting
 * to move the motors.
 */
static volatile BOOL motion_ready = FALSE;


<<<<<<< Updated upstream
/* ------------------------------------------------------------------ *
 *  One record travelling through the pipeline
 * ------------------------------------------------------------------ */

typedef struct {
	UW	seq;			/* sequence number, mutex-guarded    */
	INT	made_on;		/* processor that produced it        */
	UW	payload;		/* something to check on arrival     */
} RECORD;

/* Kernel object ids, filled in by usermain(). */
LOCAL ID	mpfid;			/* fixed-size memory pool            */
LOCAL ID	mtxid;			/* guards next_seq                   */
LOCAL ID	mbfid;			/* producer -> consumer              */
LOCAL ID	semid;			/* consumer -> producer (credits)    */
LOCAL ID	flgid;			/* both -> monitor                   */

/* Shared state.  next_seq is guarded by the mutex; the counters below are
   written by one task each and only read by the monitor, so they need no
   lock of their own. */
LOCAL UW	next_seq;
LOCAL UW	produced, consumed, dropped;
LOCAL INT	last_made_on, last_seen_on;

/* Backing store for the pool and the message buffer.  Static, because a
   kernel object must not own memory that can go out of scope. */
LOCAL UW	mpf_buf[(N_RECORDS * sizeof(RECORD) + sizeof(UW) - 1) / sizeof(UW)
			+ N_RECORDS];
LOCAL UW	mbf_buf[(CREDITS * sizeof(RECORD)) / sizeof(UW) + CREDITS * 4];

/* ------------------------------------------------------------------ *
 *  Producer -- pinned to processor 1 under SMP
 * ------------------------------------------------------------------ */

LOCAL void producer_task(INT stacd, void *exinf)
{
	RECORD	*rec;
	RECORD	msg;
	ER	er;

	(void)stacd; (void)exinf;

	while(1) {
		/* Wait for a credit: the consumer returns one per message, so
		   the producer can never outrun it by more than CREDITS. */
		er = tk_wai_sem(semid, 1, 1000);
		if(er < E_OK) { dropped++; continue; }

		/* A block from the fixed-size pool, rather than a local, to
		   show tk_get_mpf/tk_rel_mpf round-tripping. */
		er = tk_get_mpf(mpfid, (void **)&rec, 100);
		if(er < E_OK) { dropped++; tk_sig_sem(semid, 1); continue; }

		/* The sequence counter is the one piece of state two tasks
		   could race on, so it is the one thing under a mutex. */
		tk_loc_mtx(mtxid, TMO_FEVR);
		rec->seq = ++next_seq;
		tk_unl_mtx(mtxid);

		rec->made_on = THIS_PRC();
		rec->payload = rec->seq * 7U;

		msg = *rec;			/* copy out before releasing */
		tk_rel_mpf(mpfid, rec);

		er = tk_snd_mbf(mbfid, &msg, (INT)sizeof msg, 1000);
		if(er < E_OK) { dropped++; tk_sig_sem(semid, 1); continue; }

		produced++;
		last_made_on = msg.made_on;
		tk_set_flg(flgid, FLG_PRODUCED);

		tk_dly_tsk(PRODUCE_MS);
	}
}

/* ------------------------------------------------------------------ *
 *  Consumer -- pinned to processor 2 under SMP
 * ------------------------------------------------------------------ */

LOCAL void consumer_task(INT stacd, void *exinf)
{
	RECORD	msg;
	INT	sz;

	(void)stacd; (void)exinf;

	while(1) {
		sz = tk_rcv_mbf(mbfid, &msg, TMO_FEVR);
		if(sz != (INT)sizeof msg) { dropped++; continue; }

		/* Cheap integrity check: the payload is a known function of
		   the sequence number, so a torn or reordered message shows
		   up immediately. */
		if(msg.payload != msg.seq * 7U) { dropped++; continue; }

		consumed++;
		last_seen_on = THIS_PRC();
		tk_set_flg(flgid, FLG_CONSUMED);

		/* Return the credit. */
		tk_sig_sem(semid, 1);
	}
}

/* ------------------------------------------------------------------ *
 *  Monitor -- woken by the event flag, prints a line
 * ------------------------------------------------------------------ */

LOCAL void monitor_task(INT stacd, void *exinf)
{
	UINT	ptn;
	ER	er;
	INT	i;

	(void)stacd; (void)exinf;

	/* On a USB-CDC build the console is a 4 KB ring drained by the host.
	   Anything printed before the host enumerates goes into a ring nobody
	   is reading and is lost, so wait for the link first.  Bounded at ~20 s
	   so a headless board still runs; on a UART build tm_usb_state() is a
	   constant and this falls straight through. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(500);

	tm_printf((UB *)"\n=== uT-Kernel 3.0 / RP2040 demo ===\n");
#if TK_SUPPORT_SMP
	tm_printf((UB *)"SMP build: %d processors\n", TK_MAX_CORE);
#else
	tm_printf((UB *)"single-core build\n");
#endif
	tm_printf((UB *)"producer -> [mbf] -> consumer, %d credits, %d ms period\n\n",
		  CREDITS, PRODUCE_MS);

	while(1) {
		/* Wait until both ends have moved at least once since the
		   last report.  TWF_ANDW = wait for every bit; the flag is
		   cleared as part of the wait so the next round starts
		   clean. */
		er = tk_wai_flg(flgid, FLG_PRODUCED | FLG_CONSUMED,
				TWF_ANDW | TWF_CLR, &ptn, REPORT_MS);

		if(er == E_TMOUT) {
			tm_printf((UB *)"[monitor] stalled: produced=%u consumed=%u\n",
				  produced, consumed);
			continue;
		}

		tm_printf((UB *)"[monitor] seq=%u produced=%u consumed=%u dropped=%u"
			       "  made_on=prc%d seen_on=prc%d\n",
			  next_seq, produced, consumed, dropped,
			  last_made_on, last_seen_on);

		/* One line every REPORT_MS cannot overrun a 4 KB ring, but say
		   so out loud if the console ever does drop bytes. */
		if(tm_usb_dropped_bytes() != 0) {
			tm_printf((UB *)"[monitor] console dropped %u bytes\n",
				  tm_usb_dropped_bytes());
		}

		tk_dly_tsk(REPORT_MS);
	}
}

#if TM_WIFI_CYW43
/* ------------------------------------------------------------------ *
 *  WiFi status -- only built when WIFI=cyw43
 *
 *  The radio is not driven from here.  A dedicated service task owns the
 *  CYW43439, lwIP and the echo session -- pinned to processor 1 under SMP,
 *  and simply the only processor there is on a single-core build.  This task
 *  only reads the snapshot that service publishes and prints what changed.
 *  Reading is safe from any task: cyw43_utk_get_status() takes a seqlock
 *  copy, which is why the application goes through it rather than calling
 *  lwip_utk_udp_get_status() directly.
 * ------------------------------------------------------------------ */

#define WIFI_IDLE_MS	3000	/* nothing happening: report rarely      */
#define WIFI_BUSY_MS	 200	/* session running: drain the echo ring  */

/* Compose, then emit.  tm_printf is atomic per call, but a line built from
   several calls can still be spliced by a higher-priority task printing
   between them -- so every line below is formatted into a local buffer and
   emitted with exactly one call. */
LOCAL void fmt_ipv4(UB *dst, const UB *a)
{
	tm_sprintf(dst, (UB *)"%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}

LOCAL void wifi_status_task(INT stacd, void *exinf)
{
	T_CYW43_UTK_STATUS	st;
	UW			last_link = 0xffffffffU;
	UW			cursor = 0;	/* echoes printed so far */
	UB			ip[16];
	UW			last_dhcp = 0xffffffffU;
	UW			stalled_polls = 0;
	UW			shown_session = 0xffffffffU;
	UW			summarised = 0xffffffffU;
	INT			i;

	(void)stacd; (void)exinf;

	/* Same enumeration wait the monitor uses.  Without it this task races
	   ahead of the banner and its first lines land before the demo has
	   introduced itself. */
	for(i = 0; i < 200 && tm_usb_state() < 2; i++) {
		tk_dly_tsk(100);
	}
	tk_dly_tsk(700);		/* let the monitor print its banner first */

	while(1) {
		cyw43_utk_get_status(&st);

		/* Link transitions, printed once each rather than every tick. */
		if(st.link_up != last_link) {
			last_link = st.link_up;
			if(st.link_up) {
				tm_printf((UB *)"[wifi] link UP  rssi=%d  mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
					  st.link_rssi, st.mac[0], st.mac[1],
					  st.mac[2], st.mac[3], st.mac[4], st.mac[5]);
				/* The address is reported by its own transition
				   below -- DHCP usually finishes after this. */
			} else {
				tm_printf((UB *)"[wifi] link DOWN\n");
			}
		}

		/* DHCP completes some time after the link comes up, so report it
		   on its own transition.  Without this the address is only ever
		   printed in the rare case it lands in the same poll as link-up,
		   and a run that never gets an address looks like silence. */
		if(st.dhcp_complete != last_dhcp) {
			last_dhcp = st.dhcp_complete;
			if(st.dhcp_complete) {
				UB gw[16], dns[16];
				fmt_ipv4(ip, st.dhcp_address);
				fmt_ipv4(gw, st.dhcp_gateway);
				fmt_ipv4(dns, st.dhcp_dns);
				tm_printf((UB *)"[wifi] ip=%s  gw=%s  dns=%s\n",
					  ip, gw, dns);
			}
		}

		if(!st.udp_enabled) {
			tk_dly_tsk(WIFI_IDLE_MS);
			continue;
		}

		/* Nothing sent yet?  Say what is still missing rather than going
		   quiet.  The session cannot start until the interface is up and
		   has an address. */
		if(st.udp_packets_sent == 0 && st.udp_write_seq == 0) {
			if(++stalled_polls == 25U) {	/* ~5 s at the busy rate */
				tm_printf((UB *)"[udp] waiting: link=%u netif_up=%u"
					       " link_up=%u dhcp=%u addr=%u.%u.%u.%u\n",
					  st.link_up, st.netif_up, st.netif_link_up,
					  st.dhcp_complete, st.dhcp_address[0],
					  st.dhcp_address[1], st.dhcp_address[2],
					  st.dhcp_address[3]);
				stalled_polls = 0;
			}
		} else {
			stalled_polls = 0;
		}

		/* Announce the session once, when the first packet goes out.
		   Before that the target is still 0.0.0.0:0 and printing it is
		   noise, not information. */
		if(st.udp_packets_sent != 0 && st.udp_session_count != shown_session) {
			shown_session = st.udp_session_count;
			fmt_ipv4(ip, st.udp_target);
			tm_printf((UB *)"[udp] session %u: echoing to %s:%u,"
				       " %u packets of %u bytes\n",
				  st.udp_session_count + 1U, ip, st.udp_port,
				  st.udp_expected_packets, st.udp_payload_size);
		}

		/* Drain the ring: print every echo that arrived since last look.
		   If the reader ever falls more than LWIP_UTK_UDP_RING behind,
		   skip forward and say so rather than printing stale slots. */
		if(st.udp_write_seq > cursor) {
			if(st.udp_write_seq - cursor > CYW43_UDP_RING) {
				tm_printf((UB *)"[udp] (%u echoes not shown, reader behind)\n",
					  st.udp_write_seq - cursor - CYW43_UDP_RING);
				cursor = st.udp_write_seq - CYW43_UDP_RING;
			}
			while(cursor < st.udp_write_seq) {
				UW	slot = cursor % CYW43_UDP_RING;
				UB	line[96];
				INT	n;

				n = tm_sprintf(line, (UB *)"[udp] echo #%u, %u bytes:",
					       st.udp_ring_seq[slot],
					       st.udp_ring_len[slot]);
				for(i = 0; i < 12 && i < (INT)st.udp_ring_len[slot]; i++) {
					n += tm_sprintf(&line[n], (UB *)" %02x",
							st.udp_ring_payload[slot][i]);
				}
				if(st.udp_ring_len[slot] > 12) {
					n += tm_sprintf(&line[n], (UB *)" ...");
				}
				tm_printf((UB *)"%s\n", line);
				cursor++;
			}
		}

		if(st.udp_complete && st.udp_session_count != summarised) {
			summarised = st.udp_session_count;
			fmt_ipv4(ip, st.udp_target);
			tm_printf((UB *)"[udp] %s:%u  sent=%u recv=%u matched=%u/%u"
				       "  corrupt=%u retries=%u errs=%u  %u ms\n",
				  ip, st.udp_port, st.udp_packets_sent,
				  st.udp_packets_received, st.udp_packets_matched,
				  st.udp_expected_packets, st.udp_corrupt,
				  st.udp_retries, st.udp_send_errors,
				  st.udp_elapsed_ms);
			tm_printf((UB *)"[udp] session %u complete: result=%d%s\n",
				  st.udp_session_count + 1U, st.udp_result,
				  (st.udp_packets_matched == st.udp_expected_packets
				   && st.udp_corrupt == 0)
				  ? "  ALL PACKETS ECHOED" : "");
		}

		/* Poll fast whenever a session could be running, not only once
		   one has been observed running.  A session lasts about 1.4 s
		   and repeats every 5 s; at the idle rate this task would sleep
		   straight through one and find the ring already wrapped.  Once
		   the link is up and UDP is enabled, stay at the fast rate. */
		tk_dly_tsk((st.udp_enabled && st.link_up)
			   ? WIFI_BUSY_MS : WIFI_IDLE_MS);
	}
}

LOCAL T_CTSK ctsk_wifi = {
	.itskpri	= 8,
	.stksz		= STACK_SZ,
	.task		= wifi_status_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};
#endif	/* TM_WIFI_CYW43 */

/* ------------------------------------------------------------------ *
 *  Task and object definitions
 * ------------------------------------------------------------------ */

LOCAL T_CTSK ctsk_producer = {
	.itskpri	= 5,
	.stksz		= STACK_SZ,
	.task		= producer_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC1,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_consumer = {
	.itskpri	= 6,
	.stksz		= STACK_SZ,
	.task		= consumer_task,
#if TK_SUPPORT_SMP
	.tskatr		= TA_HLNG | TA_RNG3 | TA_ASSPRC,
	.assprc		= TP_PRC2,
#else
	.tskatr		= TA_HLNG | TA_RNG3,
#endif
};

LOCAL T_CTSK ctsk_monitor = {
	.itskpri	= 7,
	.stksz		= STACK_SZ,
	.task		= monitor_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL T_CTSK ctsk_blink = {
	.itskpri	= 10,
	.stksz		= STACK_SZ,
	.task		= blink_task,
	.tskatr		= TA_HLNG | TA_RNG3,
};

LOCAL T_CMPF cmpf = {
	.mpfatr		= TA_TFIFO | TA_RNG3,
	.mpfcnt		= N_RECORDS,
	.blfsz		= sizeof(RECORD),
	.bufptr		= mpf_buf,
};

LOCAL T_CMTX cmtx = {
	.mtxatr		= TA_TFIFO | TA_INHERIT,
};

LOCAL T_CMBF cmbf = {
	.mbfatr		= TA_TFIFO,
	.bufsz		= sizeof mbf_buf,
	.maxmsz		= sizeof(RECORD),
	.bufptr		= mbf_buf,
};

LOCAL T_CSEM csem = {
	.sematr		= TA_TFIFO,
	.isemcnt	= CREDITS,
	.maxsem		= CREDITS,
};

LOCAL T_CFLG cflg = {
	.flgatr		= TA_TFIFO | TA_WMUL,
	.iflgptn	= 0,
};

/* ------------------------------------------------------------------ *
 *  Entry point
 * ------------------------------------------------------------------ */

/* Create an object, or say which one failed and stop.  A silent object
   failure here would surface much later as a mysterious hang. */
LOCAL BOOL made(const char *what, ID id)
=======
/* ================================================================
 * USB SERIAL MONITOR WAIT
 * ================================================================
 *
 * The Pico can start executing tasks before Windows has finished
 * detecting the USB serial connection.
 *
 * Waiting briefly here makes it less likely that the first status
 * messages disappear before Serial Monitor connects.
 */
static void wait_for_console(void)
{
    INT i;

    for (i = 0; i < 100 && tm_usb_state() < 2; i++)
    {
        tk_dly_tsk(50);
    }
}


/* ================================================================
 * MOTION / ENCODER TASK
 * ================================================================
 *
 * Buddy 2 hardware baseline.
 *
 * This task:
 *   1. Initialises the motor driver.
 *   2. Ensures the motors begin stopped.
 *   3. Continuously updates encoder counts.
 *
 * It DOES NOT make the robot drive automatically.
 */
LOCAL void motion_io_task(INT stacd, void *exinf)
{
    ER err;

    (void)stacd;
    (void)exinf;

    wait_for_console();

    /*
     * Keep trying until motor/encoder initialisation succeeds.
     */
    do
    {
        err = motion_init();

        if (err < E_OK)
        {
            tm_printf(
                (UB *)"[MOTION] init failed err=%d; retrying\n",
                err
            );

            tk_dly_tsk(1000);
        }

    } while (err < E_OK);


    motion_ready = TRUE;

    tm_printf(
        (UB *)"[MOTION] ready: motors stopped, encoder polling active\n"
    );


    /*
     * Continuously sample the wheel encoders.
     */
    while (1)
    {
        encoder_update();

        tk_dly_tsk(ROBOT_ENCODER_SAMPLE_MS);
    }
}


/* ================================================================
 * IR + ULTRASONIC + SERVO TASK
 * ================================================================
 *
 * Buddy 3 and Buddy 5 hardware baseline.
 *
 * This task continuously displays:
 *
 *   IR Left
 *   IR Centre
 *   IR Right
 *   Ultrasonic distance
 *   Left encoder count
 *   Right encoder count
 */
LOCAL void sensor_task(INT stacd, void *exinf)
{
    LineSensorData line = {0, 0, 0};

    EncoderData enc;

    float distance;

    ER line_err;
    ER err;


    (void)stacd;
    (void)exinf;


    wait_for_console();


    /* ------------------------------------------------------------
     * Initialise IR sensors
     * ------------------------------------------------------------
     */

    do
    {
        err = line_sensor_init();

        if (err < E_OK)
        {
            tm_printf(
                (UB *)"[SENSOR] line_sensor_init failed err=%d; retrying\n",
                err
            );

            tk_dly_tsk(1000);
        }

    } while (err < E_OK);


    /* ------------------------------------------------------------
     * Initialise ultrasonic + servo
     * ------------------------------------------------------------
     */

    do
    {
        err = obstacle_init();

        if (err < E_OK)
        {
            tm_printf(
                (UB *)"[SENSOR] obstacle_init failed err=%d; retrying\n",
                err
            );

            tk_dly_tsk(1000);
        }

    } while (err < E_OK);


    /* ------------------------------------------------------------
     * OPTIONAL SERVO SELF TEST
     *
     * Disabled by default in app_config.h.
     * ------------------------------------------------------------
     */

#if ROBOT_RUN_SERVO_SELF_TEST

    tm_printf(
        (UB *)"[SERVO] one-shot test: 45 -> 90 -> 135 -> 90 deg\n"
    );

    (void)servo_set_angle(45);

    tk_dly_tsk(700);


    (void)servo_set_angle(90);

    tk_dly_tsk(700);


    (void)servo_set_angle(135);

    tk_dly_tsk(700);


    (void)servo_set_angle(90);

#endif


    tm_printf(
        (UB *)"[SENSOR] ready: IR + ultrasonic + servo centre\n"
    );


    /* ------------------------------------------------------------
     * Continuous hardware readings
     * ------------------------------------------------------------
     */

    while (1)
    {
        /*
         * Read all three IR sensors.
         */
        line_err = line_sensor_read_all(&line);


        /*
         * Measure ultrasonic distance.
         */
        distance = ultrasonic_read_cm();


        /*
         * Obtain latest encoder counts.
         */
        enc = encoder_get_counts();


        /*
         * IR failed.
         */
        if (line_err < E_OK)
        {
            tm_printf(
                (UB *)
                "[SENSOR] IR read error=%d | ENC L=%d R=%d\n",

                line_err,
                enc.left,
                enc.right
            );
        }

        /*
         * Ultrasonic timed out.
         */
        else if (distance < 0.0f)
        {
            tm_printf(
                (UB *)
                "[SENSOR] IR L=%d C=%u R=%u | "
                "US=TIMEOUT | ENC L=%d R=%d\n",

                line.left,
                (UINT)line.center,
                (UINT)line.right,
                enc.left,
                enc.right
            );
        }

        /*
         * Normal sensor readings.
         */
        else
        {
            tm_printf(
                (UB *)
                "[SENSOR] IR L=%d C=%u R=%u | "
                "US=%d cm | ENC L=%d R=%d\n",

                line.left,
                (UINT)line.center,
                (UINT)line.right,
                (INT)distance,
                enc.left,
                enc.right
            );
        }


        tk_dly_tsk(ROBOT_SENSOR_PERIOD_MS);
    }
}


/* ================================================================
 * IMU TASK
 * ================================================================
 *
 * Buddy 4 hardware baseline.
 *
 * Important difference from your old code:
 *
 * We DO NOT automatically print
 *
 *      X=0 Y=0 Z=0
 *
 * when the I2C communication fails.
 *
 * The return code from the IMU driver is checked.
 */
LOCAL void imu_task(INT stacd, void *exinf)
{
    IMU_AccelData accel;

    ER err;

    UW shown_addr = 0xFFFFFFFFU;


    (void)stacd;
    (void)exinf;


    wait_for_console();


    while (1)
    {
        /* --------------------------------------------------------
         * IMU not connected/initialised
         * --------------------------------------------------------
         */

        if (!imu_is_ready())
        {
            err = imu_init();


            if (err < E_OK)
            {
                tm_printf(
                    (UB *)
                    "[IMU] init/I2C failed err=%d "
                    "(GP0=SDA GP1=SCL); retrying\n",

                    err
                );


                tk_dly_tsk(1000);

                continue;
            }


            /*
             * Print the detected accelerometer address.
             *
             * Usually:
             *
             * 0x19
             *
             * Some boards may use:
             *
             * 0x18
             */
            if (shown_addr != imu_get_address())
            {
                shown_addr = imu_get_address();

                tm_printf(
                    (UB *)
                    "[IMU] ready on I2C address 0x%02x\n",

                    shown_addr
                );
            }
        }


        /* --------------------------------------------------------
         * Read accelerometer
         * --------------------------------------------------------
         */

        err = imu_read_accel(&accel);


        /*
         * VERY IMPORTANT:
         *
         * An I2C error is reported as an ERROR.
         *
         * It is no longer displayed as fake:
         *
         * X=0 Y=0 Z=0
         */
        if (err < E_OK)
        {
            tm_printf(
                (UB *)
                "[IMU] read failed err=%d "
                "total_errors=%u; reconnecting\n",

                err,
                imu_get_error_count()
            );


            tk_dly_tsk(500);

            continue;
        }


        /* --------------------------------------------------------
         * Successful accelerometer reading
         * --------------------------------------------------------
         */

        tm_printf(
            (UB *)
            "[IMU] X=%d Y=%d Z=%d\n",

            accel.x,
            accel.y,
            accel.z
        );


        tk_dly_tsk(ROBOT_IMU_PERIOD_MS);
    }
}


/* ================================================================
 * OPTIONAL MOTOR SELF TEST
 * ================================================================
 *
 * This entire task is excluded when:
 *
 * ROBOT_RUN_MOTOR_SELF_TEST == 0
 *
 * Keep it disabled during our first test.
 */

#if ROBOT_RUN_MOTOR_SELF_TEST

LOCAL void motor_self_test_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;


    wait_for_console();


    /*
     * Wait until the main motion task has initialised the motor driver.
     */
    while (!motion_ready)
    {
        tk_dly_tsk(50);
    }


    tm_printf(
        (UB *)
        "[MOTOR TEST] starting in 3 seconds - "
        "keep wheels off ground\n"
    );


    tk_dly_tsk(3000);


    /* ------------------------------------------------------------
     * LEFT FORWARD
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] left forward %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_left(MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();

    tk_dly_tsk(700);


    /* ------------------------------------------------------------
     * RIGHT FORWARD
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] right forward %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_right(MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();

    tk_dly_tsk(700);


    /* ------------------------------------------------------------
     * LEFT REVERSE
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] left reverse %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_left(-MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();

    tk_dly_tsk(700);


    /* ------------------------------------------------------------
     * RIGHT REVERSE
     * ------------------------------------------------------------
     */

    tm_printf(
        (UB *)
        "[MOTOR TEST] right reverse %d%%\n",
        MOTOR_SELF_TEST_SPEED
    );


    motor_set_right(-MOTOR_SELF_TEST_SPEED);

    tk_dly_tsk(800);

    motor_stop();


    tm_printf(
        (UB *)
        "[MOTOR TEST] finished; motors stopped\n"
    );


    /*
     * Test only runs once.
     */
    tk_slp_tsk(TMO_FEVR);
}

#endif


/* ================================================================
 * RTOS TASK CONFIGURATION
 * ================================================================
 */


/*
 * Motor + encoder task
 */
LOCAL T_CTSK ctsk_motion_io =
{
    .itskpri = 6,
    .stksz   = STACK_SZ,
    .task    = motion_io_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


/*
 * IMU task
 */
LOCAL T_CTSK ctsk_imu =
{
    .itskpri = 8,
    .stksz   = STACK_SZ,
    .task    = imu_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


/*
 * IR + ultrasonic + encoder display
 */
LOCAL T_CTSK ctsk_sensor =
{
    .itskpri = 9,
    .stksz   = STACK_SZ,
    .task    = sensor_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


/*
 * Existing Pico LED heartbeat task
 */
LOCAL T_CTSK ctsk_blink =
>>>>>>> Stashed changes
{
    .itskpri = 10,
    .stksz   = STACK_SZ,
    .task    = blink_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};


#if ROBOT_RUN_MOTOR_SELF_TEST

LOCAL T_CTSK ctsk_motor_test =
{
    .itskpri = 7,
    .stksz   = STACK_SZ,
    .task    = motor_self_test_task,
    .tskatr  = TA_HLNG | TA_RNG3,
};

#endif


/* ================================================================
 * TASK STARTING HELPER
 * ================================================================
 */

static BOOL start_task(const char *name, T_CTSK *cfg)
{
    ID tid;

    ER err;


    /*
     * Create task.
     */
    tid = tk_cre_tsk(cfg);


    if (tid <= E_OK)
    {
        tm_printf(
            (UB *)
            "[INIT] task %s create failed err=%d\n",

            name,
            tid
        );


        return FALSE;
    }


    /*
     * Start task.
     */
    err = tk_sta_tsk(tid, 0);


    if (err < E_OK)
    {
        tm_printf(
            (UB *)
            "[INIT] task %s start failed err=%d\n",

            name,
            err
        );


        return FALSE;
    }


    return TRUE;
}


/* ================================================================
 * APPLICATION ENTRY POINT
 * ================================================================
 */

EXPORT INT usermain(void)
{
<<<<<<< Updated upstream
	ID	tid;

	mpfid = tk_cre_mpf(&cmpf); if(!made("mpf", mpfid)) return 1;
	mtxid = tk_cre_mtx(&cmtx); if(!made("mtx", mtxid)) return 1;
	mbfid = tk_cre_mbf(&cmbf); if(!made("mbf", mbfid)) return 1;
	semid = tk_cre_sem(&csem); if(!made("sem", semid)) return 1;
	flgid = tk_cre_flg(&cflg); if(!made("flg", flgid)) return 1;

	tid = tk_cre_tsk(&ctsk_blink);    if(made("tsk(blink)",    tid)) tk_sta_tsk(tid, 0);
	tid = tk_cre_tsk(&ctsk_monitor);  if(made("tsk(monitor)",  tid)) tk_sta_tsk(tid, 0);
	tid = tk_cre_tsk(&ctsk_consumer); if(made("tsk(consumer)", tid)) tk_sta_tsk(tid, 0);
	tid = tk_cre_tsk(&ctsk_producer); if(made("tsk(producer)", tid)) tk_sta_tsk(tid, 0);
#if TM_WIFI_CYW43
	tid = tk_cre_tsk(&ctsk_wifi);     if(made("tsk(wifi)",     tid)) tk_sta_tsk(tid, 0);
=======
    /*
     * Start Pico heartbeat LED.
     */
    (void)start_task(
        "blink",
        &ctsk_blink
    );


    /*
     * Start motor/encoder hardware task.
     */
    (void)start_task(
        "motion_io",
        &ctsk_motion_io
    );


    /*
     * Start IMU task.
     */
    (void)start_task(
        "imu",
        &ctsk_imu
    );


    /*
     * Start IR / ultrasonic / encoder display task.
     */
    (void)start_task(
        "sensor",
        &ctsk_sensor
    );


    /*
     * Optional motor movement test.
     *
     * It will not be compiled when
     * ROBOT_RUN_MOTOR_SELF_TEST = 0.
     */
#if ROBOT_RUN_MOTOR_SELF_TEST

    (void)start_task(
        "motor_test",
        &ctsk_motor_test
    );

>>>>>>> Stashed changes
#endif


    /*
     * VERY IMPORTANT:
     *
     * usermain must not return.
     *
     * All worker tasks have already been started above,
     * so the initial application task can now sleep forever.
     *
     * Unlike your previous app_main.c, there is NOTHING after
     * this sleep that still needs to execute.
     */
    tk_slp_tsk(TMO_FEVR);


    return 0;
}