#ifndef _LINUX_HMP_BOOST_H
#define _LINUX_HMP_BOOST_H

enum  hmp_boost_state {
	HMP_BOOST_NONE = 0,
	HMP_BOOST_FULL_ENABLE = 1,
	HMP_BOOST_SEMI_ENABLE = 2,
	HMP_BOOST_FULL_DISABLE = -1,
	HMP_BOOST_SEMI_DISABLE = -2,
};

#define MAX_NUM_HMP_BOOST_STATE 3 /* HMP_BOOST_SEMI_ENABLE + 1 */

#define MIN_HMP_BOOST_STATE HMP_BOOST_SEMI_DISABLE
#define MAX_HMP_BOOST_STATE HMP_BOOST_SEMI_ENABLE

extern int sched_hmp_boost(int);
extern int sched_hmp_boost_on(int);

#endif
