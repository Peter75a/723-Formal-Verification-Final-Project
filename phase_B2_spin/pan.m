#define rand	pan_rand
#define pthread_equal(a,b)	((a)==(b))
#if defined(HAS_CODE) && defined(VERBOSE)
	#ifdef BFS_PAR
		bfs_printf("Pr: %d Tr: %d\n", II, t->forw);
	#else
		cpu_printf("Pr: %d Tr: %d\n", II, t->forw);
	#endif
#endif
	switch (t->forw) {
	default: Uerror("bad forward move");
	case 0:	/* if without executable clauses */
		continue;
	case 1: /* generic 'goto' or 'skip' */
		IfNotBlocked
		_m = 3; goto P999;
	case 2: /* generic 'else' */
		IfNotBlocked
		if (trpt->o_pm&1) continue;
		_m = 3; goto P999;

		 /* PROC Check */
	case 3: // STATE 1 - example2_traffic_light.pml:20 - [assert(((light==NS_GREEN)||(light==EW_GREEN)))] (0:0:0 - 1)
		IfNotBlocked
		reached[2][1] = 1;
		spin_assert(((now.light==2)||(now.light==1)), "((light==2)||(light==1))", II, tt, t);
		_m = 3; goto P999; /* 0 */
	case 4: // STATE 5 - example2_traffic_light.pml:22 - [-end-] (0:0:0 - 1)
		IfNotBlocked
		reached[2][5] = 1;
		if (!delproc(1, II)) continue;
		_m = 3; goto P999; /* 0 */

		 /* PROC Controller */
	case 5: // STATE 1 - example2_traffic_light.pml:13 - [light = NS_GREEN] (0:0:1 - 1)
		IfNotBlocked
		reached[1][1] = 1;
		(trpt+1)->bup.oval = now.light;
		now.light = 2;
#ifdef VAR_RANGES
		logval("light", now.light);
#endif
		;
		_m = 3; goto P999; /* 0 */
	case 6: // STATE 2 - example2_traffic_light.pml:14 - [light = EW_GREEN] (0:0:1 - 1)
		IfNotBlocked
		reached[1][2] = 1;
		(trpt+1)->bup.oval = now.light;
		now.light = 1;
#ifdef VAR_RANGES
		logval("light", now.light);
#endif
		;
		_m = 3; goto P999; /* 0 */
	case 7: // STATE 6 - example2_traffic_light.pml:16 - [-end-] (0:0:0 - 1)
		IfNotBlocked
		reached[1][6] = 1;
		if (!delproc(1, II)) continue;
		_m = 3; goto P999; /* 0 */

		 /* PROC :init: */
	case 8: // STATE 1 - example2_traffic_light.pml:6 - [light = NS_GREEN] (0:0:1 - 1)
		IfNotBlocked
		reached[0][1] = 1;
		(trpt+1)->bup.oval = now.light;
		now.light = 2;
#ifdef VAR_RANGES
		logval("light", now.light);
#endif
		;
		_m = 3; goto P999; /* 0 */
	case 9: // STATE 2 - example2_traffic_light.pml:7 - [(run Controller())] (0:0:0 - 1)
		IfNotBlocked
		reached[0][2] = 1;
		if (!(addproc(II, 1, 1)))
			continue;
		_m = 3; goto P999; /* 0 */
	case 10: // STATE 3 - example2_traffic_light.pml:8 - [(run Check())] (0:0:0 - 1)
		IfNotBlocked
		reached[0][3] = 1;
		if (!(addproc(II, 1, 2)))
			continue;
		_m = 3; goto P999; /* 0 */
	case 11: // STATE 4 - example2_traffic_light.pml:9 - [-end-] (0:0:0 - 1)
		IfNotBlocked
		reached[0][4] = 1;
		if (!delproc(1, II)) continue;
		_m = 3; goto P999; /* 0 */
	case  _T5:	/* np_ */
		if (!((!(trpt->o_pm&4) && !(trpt->tau&128))))
			continue;
		/* else fall through */
	case  _T2:	/* true */
		_m = 3; goto P999;
#undef rand
	}

