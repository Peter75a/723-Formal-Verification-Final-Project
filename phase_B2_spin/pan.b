	switch (t->back) {
	default: Uerror("bad return move");
	case  0: goto R999; /* nothing to undo */

		 /* PROC Check */
;
		;
		
	case 4: // STATE 5
		;
		p_restor(II);
		;
		;
		goto R999;

		 /* PROC Controller */

	case 5: // STATE 1
		;
		now.light = trpt->bup.oval;
		;
		goto R999;

	case 6: // STATE 2
		;
		now.light = trpt->bup.oval;
		;
		goto R999;

	case 7: // STATE 6
		;
		p_restor(II);
		;
		;
		goto R999;

		 /* PROC :init: */

	case 8: // STATE 1
		;
		now.light = trpt->bup.oval;
		;
		goto R999;

	case 9: // STATE 2
		;
		;
		delproc(0, now._nr_pr-1);
		;
		goto R999;

	case 10: // STATE 3
		;
		;
		delproc(0, now._nr_pr-1);
		;
		goto R999;

	case 11: // STATE 4
		;
		p_restor(II);
		;
		;
		goto R999;
	}

