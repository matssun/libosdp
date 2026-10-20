/*
 * Copyright (c) 2026 Mats Sundvall
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <unistd.h>

#include <osdp.h>
#include "test.h"

/*
 * Offline retry. Once the CP declares a PD offline it leaves it alone for a
 * while before trying again. That wait starts at OSDP_ONLINE_RETRY_WAIT_INIT_MS
 * and doubles on every further failure up to OSDP_ONLINE_RETRY_WAIT_MAX_MS, and
 * a PD that comes online starts over. A PD that only dropped off for a moment
 * (a reader reset, a cable reseated, a power blip) is therefore tried again
 * within about a second instead of after the maximum wait.
 *
 * Time is simulated by rewinding pd->tstamp, so the suite does not sleep.
 */

extern void test_state_update(struct osdp_pd *);

/* Canned PD for the INIT -> CAPDET -> ONLINE handshake (see test-cp-fsm.c). */
extern int test_cp_fsm_send(void *data, uint8_t *buf, int len);
extern int test_cp_fsm_receive(void *data, uint8_t *buf, int len);
#ifdef OPT_OSDP_RX_ZERO_COPY
extern int test_cp_fsm_recv_pkt(void *data, const uint8_t **buf, int *max_len);
extern void test_cp_fsm_release_pkt(void *data, const uint8_t *buf);
#endif

/* While false the PD is unplugged: it never answers. */
static bool s_pd_present;

#ifndef OPT_OSDP_RX_ZERO_COPY
static int offline_receive(void *data, uint8_t *buf, int len)
{
	if (!s_pd_present) {
		return -1;
	}
	return test_cp_fsm_receive(data, buf, len);
}
#else
static int offline_recv_pkt(void *data, const uint8_t **buf, int *max_len)
{
	if (!s_pd_present) {
		return -1;
	}
	return test_cp_fsm_recv_pkt(data, buf, max_len);
}
#endif

static struct osdp *offline_setup(struct test *t)
{
	struct osdp_channel channel = {
		.data = NULL,
		.send = test_cp_fsm_send,
#ifndef OPT_OSDP_RX_ZERO_COPY
		.recv = offline_receive,
#else
		.recv_pkt = offline_recv_pkt,
		.release_pkt = test_cp_fsm_release_pkt,
#endif
		.flush = NULL,
	};
	osdp_pd_info_t info = {
		.address = 101,
		.baud_rate = 9600,
		.flags = 0,
		.scbk = NULL,
	};
	struct osdp *ctx;

#ifndef OPT_OSDP_LOG_MINIMAL
	osdp_logger_init("osdp::cp", t->loglevel, NULL);
#endif
	ctx = (struct osdp *)osdp_cp_setup(&channel, 1, &info);
	if (ctx == NULL) {
		printf(SUB_2 "osdp_cp_setup() failed\n");
		return NULL;
	}
	SET_CURRENT_PD(ctx, 0);
	SET_FLAG(GET_CURRENT_PD(ctx), PD_FLAG_SKIP_SEQ_CHECK);
	return ctx;
}

static bool offline_reach_online(struct osdp_pd *pd)
{
	int i;

	s_pd_present = true;
	for (i = 0; i < 2000; i++) {
		test_state_update(pd);
		if (pd->state == OSDP_CP_STATE_ONLINE) {
			return true;
		}
		usleep(1000);
	}
	printf(SUB_2 "PD never came online\n");
	return false;
}

/* Fail the PD from whatever state it is in and check it went offline. */
static bool offline_fail(struct osdp_pd *pd)
{
	s_pd_present = false;
	pd->phy_state = OSDP_CP_PHY_STATE_ERR;
	test_state_update(pd);
	if (pd->state != OSDP_CP_STATE_OFFLINE) {
		printf(SUB_2 "expected OFFLINE after a failure, state is %d\n",
		       pd->state);
		return false;
	}
	return true;
}

/*
 * The CP must still be offline just before @wait_ms have passed and must leave
 * the offline state just after. On success the PD is left in INIT.
 */
static bool offline_wait_is(struct osdp_pd *pd, int64_t wait_ms)
{
	tick_t now = osdp_millis_now();

	pd->tstamp = now - (wait_ms - 10);
	test_state_update(pd);
	if (pd->state != OSDP_CP_STATE_OFFLINE) {
		printf(SUB_2 "left OFFLINE before %lld ms\n", (long long)wait_ms);
		return false;
	}

	pd->tstamp = now - (wait_ms + 10);
	test_state_update(pd);
	if (pd->state == OSDP_CP_STATE_OFFLINE) {
		printf(SUB_2 "still OFFLINE after %lld ms\n", (long long)wait_ms);
		return false;
	}
	return true;
}

/* The first outage is retried after the initial wait. */
static bool test_offline_initial_wait(struct test *t)
{
	bool ok;
	struct osdp *ctx = offline_setup(t);
	struct osdp_pd *pd;

	if (ctx == NULL) {
		return false;
	}
	pd = GET_CURRENT_PD(ctx);

	/*
	 * A command retry leaves its own delay in pd->wait_ms; the offline wait
	 * must not be derived from it.
	 */
	ok = offline_reach_online(pd);
	if (ok) {
		pd->wait_ms = OSDP_CMD_RETRY_WAIT_MS;
		ok = offline_fail(pd) &&
		     offline_wait_is(pd, OSDP_ONLINE_RETRY_WAIT_INIT_MS);
	}

	osdp_cp_teardown(ctx);
	return ok;
}

/* Further outages double the wait, up to the maximum, and stay there. */
static bool test_offline_backoff_ladder(struct test *t)
{
	int cycle;
	bool ok;
	int64_t wait_ms = OSDP_ONLINE_RETRY_WAIT_INIT_MS;
	struct osdp *ctx = offline_setup(t);
	struct osdp_pd *pd;

	if (ctx == NULL) {
		return false;
	}
	pd = GET_CURRENT_PD(ctx);

	ok = offline_reach_online(pd);
	/* Enough rounds to reach the maximum and stay there for a few more. */
	for (cycle = 0; ok && cycle < 24; cycle++) {
		ok = offline_fail(pd) && offline_wait_is(pd, wait_ms);
		if (!ok) {
			printf(SUB_2 "failed in cycle %d\n", cycle);
		}
		wait_ms *= 2;
		if (wait_ms > OSDP_ONLINE_RETRY_WAIT_MAX_MS) {
			wait_ms = OSDP_ONLINE_RETRY_WAIT_MAX_MS;
		}
	}

	osdp_cp_teardown(ctx);
	return ok;
}

/* Coming online again starts the sequence over from the initial wait. */
static bool test_offline_online_resets_backoff(struct test *t)
{
	int cycle;
	bool ok;
	struct osdp *ctx = offline_setup(t);
	struct osdp_pd *pd;

	if (ctx == NULL) {
		return false;
	}
	pd = GET_CURRENT_PD(ctx);

	ok = offline_reach_online(pd);
	/* Build the wait up well past the initial value... */
	for (cycle = 0; ok && cycle < 4; cycle++) {
		ok = offline_fail(pd) &&
		     offline_wait_is(pd, (int64_t)OSDP_ONLINE_RETRY_WAIT_INIT_MS
						  << cycle);
	}
	/* ...then let the PD return; the next outage must start from scratch. */
	ok = ok && offline_reach_online(pd) && offline_fail(pd) &&
	     offline_wait_is(pd, OSDP_ONLINE_RETRY_WAIT_INIT_MS);

	osdp_cp_teardown(ctx);
	return ok;
}

void run_cp_offline_tests(struct test *t)
{
	printf("\nStarting CP offline retry tests\n");

	TEST_CASE(t, "initial_wait", test_offline_initial_wait(t));
	TEST_CASE(t, "backoff_ladder", test_offline_backoff_ladder(t));
	TEST_CASE(t, "online_resets_backoff",
		  test_offline_online_resets_backoff(t));
}
