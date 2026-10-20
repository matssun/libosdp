/*
 * Copyright (c) 2026 Mats Sundvall
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <unistd.h>

#include <osdp.h>
#include "test.h"

/*
 * Offline retry: once the CP declares a PD offline it must wait before it tries
 * again, starting at OSDP_ONLINE_RETRY_WAIT_INIT_MS and doubling on every
 * further failure up to OSDP_ONLINE_RETRY_WAIT_MAX_MS. Reaching ONLINE again
 * starts the sequence over. A PD that dropped off for a moment (a reader reset,
 * a power blip) must therefore be polled again within about a second, not after
 * the maximum wait.
 *
 * Time is simulated by rewinding pd->tstamp, so the test does not sleep.
 */

extern int (*test_state_update)(struct osdp_pd *);

/* Canned PD replies for the INIT -> CAPDET -> ONLINE handshake. */
extern int test_cp_fsm_send(void *data, uint8_t *buf, int len);
extern int test_cp_fsm_receive(void *data, uint8_t *buf, int len);

static bool test_offline_pd_present;

static int test_offline_receive(void *data, uint8_t *buf, int len)
{
	if (!test_offline_pd_present)
		return -1;
	return test_cp_fsm_receive(data, buf, len);
}

static int test_offline_setup(struct test *t)
{
	osdp_pd_info_t info = {
		.address = 101,
		.baud_rate = 9600,
		.flags = 0,
		.channel.data = NULL,
		.channel.send = test_cp_fsm_send,
		.channel.recv = test_offline_receive,
		.channel.flush = NULL,
		.scbk = NULL,
	};
	struct osdp *ctx;

	osdp_logger_init("osdp::cp", t->loglevel, NULL);
	ctx = (struct osdp *)osdp_cp_setup(1, &info);
	if (ctx == NULL) {
		printf(SUB_1 "init failed!\n");
		return -1;
	}
	SET_CURRENT_PD(ctx, 0);
	SET_FLAG(GET_CURRENT_PD(ctx), PD_FLAG_SKIP_SEQ_CHECK);
	t->mock_data = (void *)ctx;
	return 0;
}

static bool test_offline_reach_online(struct osdp_pd *pd)
{
	int i;

	test_offline_pd_present = true;
	for (i = 0; i < 2000; i++) {
		test_state_update(pd);
		if (pd->state == OSDP_CP_STATE_ONLINE)
			return true;
		usleep(1000);
	}
	printf(SUB_2 "PD never came online\n");
	return false;
}

/* Fail the PD from its current state and check it went offline. */
static bool test_offline_fail(struct osdp_pd *pd)
{
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
static bool test_offline_wait_is(struct osdp_pd *pd, int64_t wait_ms)
{
	int64_t now = osdp_millis_now();

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

/*
 * Walk the rest of the backoff ladder: 2*INIT, 4*INIT, ... up to MAX and beyond.
 * The INIT step itself is the first outage, checked before this is called.
 */
static bool test_offline_ladder(struct osdp_pd *pd)
{
	int cycle;
	int64_t wait_ms = 2 * OSDP_ONLINE_RETRY_WAIT_INIT_MS;

	for (cycle = 0; cycle < 12; cycle++) {
		if (!test_offline_fail(pd) || !test_offline_wait_is(pd, wait_ms)) {
			printf(SUB_2 "failed in cycle %d\n", cycle);
			return false;
		}
		wait_ms *= 2;
		if (wait_ms > OSDP_ONLINE_RETRY_WAIT_MAX_MS)
			wait_ms = OSDP_ONLINE_RETRY_WAIT_MAX_MS;
	}
	return true;
}

void run_cp_offline_tests(struct test *t)
{
	bool result = true;
	struct osdp *ctx;
	struct osdp_pd *pd;

	printf("\nStarting CP offline retry tests\n");

	if (test_offline_setup(t))
		return;

	ctx = t->mock_data;
	pd = GET_CURRENT_PD(ctx);

	printf(SUB_1 "bringing the PD online\n");
	result = test_offline_reach_online(pd);

	if (result) {
		/*
		 * A command retry leaves its own delay in pd->wait_ms; the first
		 * offline wait must not be derived from it.
		 */
		pd->wait_ms = OSDP_CMD_RETRY_WAIT_MS;
		printf(SUB_1 "first outage retries after the initial wait\n");
		result = test_offline_fail(pd) &&
			 test_offline_wait_is(pd, OSDP_ONLINE_RETRY_WAIT_INIT_MS);
	}

	if (result) {
		printf(SUB_1 "further outages back off up to the maximum\n");
		result = test_offline_ladder(pd);
	}

	if (result) {
		/* The PD comes back: the next outage starts from scratch. */
		printf(SUB_1 "coming online resets the backoff\n");
		result = test_offline_reach_online(pd) &&
			 test_offline_fail(pd) &&
			 test_offline_wait_is(pd, OSDP_ONLINE_RETRY_WAIT_INIT_MS);
	}

	printf(SUB_1 "offline retry test %s\n", result ? "succeeded" : "failed");

	TEST_REPORT(t, result);

	osdp_cp_teardown(ctx);
}
