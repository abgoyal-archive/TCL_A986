
/* tblcmp - table compression routines */


/* $Header: /home/daffy/u0/vern/flex/RCS/tblcmp.c,v 2.11 94/11/05 17:08:28 vern Exp $ */

#include "flexdef.h"


/* declarations for functions that have forward references */

void mkentry PROTO((register int*, int, int, int, int));
void mkprot PROTO((int[], int, int));
void mktemplate PROTO((int[], int, int));
void mv2front PROTO((int));
int tbldiff PROTO((int[], int, int[]));



void bldtbl( state, statenum, totaltrans, comstate, comfreq )
int state[], statenum, totaltrans, comstate, comfreq;
	{
	int extptr, extrct[2][CSIZE + 1];
	int mindiff, minprot, i, d;

	/* If extptr is 0 then the first array of extrct holds the result
	 * of the "best difference" to date, which is those transitions
	 * which occur in "state" but not in the proto which, to date,
	 * has the fewest differences between itself and "state".  If
	 * extptr is 1 then the second array of extrct hold the best
	 * difference.  The two arrays are toggled between so that the
	 * best difference to date can be kept around and also a difference
	 * just created by checking against a candidate "best" proto.
	 */

	extptr = 0;

	/* If the state has too few out-transitions, don't bother trying to
	 * compact its tables.
	 */

	if ( (totaltrans * 100) < (numecs * PROTO_SIZE_PERCENTAGE) )
		mkentry( state, numecs, statenum, JAMSTATE, totaltrans );

	else
		{
		/* "checkcom" is true if we should only check "state" against
		 * protos which have the same "comstate" value.
		 */
		int checkcom =
			comfreq * 100 > totaltrans * CHECK_COM_PERCENTAGE;

		minprot = firstprot;
		mindiff = totaltrans;

		if ( checkcom )
			{
			/* Find first proto which has the same "comstate". */
			for ( i = firstprot; i != NIL; i = protnext[i] )
				if ( protcomst[i] == comstate )
					{
					minprot = i;
					mindiff = tbldiff( state, minprot,
							extrct[extptr] );
					break;
					}
			}

		else
			{
			/* Since we've decided that the most common destination
			 * out of "state" does not occur with a high enough
			 * frequency, we set the "comstate" to zero, assuring
			 * that if this state is entered into the proto list,
			 * it will not be considered a template.
			 */
			comstate = 0;

			if ( firstprot != NIL )
				{
				minprot = firstprot;
				mindiff = tbldiff( state, minprot,
						extrct[extptr] );
				}
			}

		/* We now have the first interesting proto in "minprot".  If
		 * it matches within the tolerances set for the first proto,
		 * we don't want to bother scanning the rest of the proto list
		 * to see if we have any other reasonable matches.
		 */

		if ( mindiff * 100 > totaltrans * FIRST_MATCH_DIFF_PERCENTAGE )
			{
			/* Not a good enough match.  Scan the rest of the
			 * protos.
			 */
			for ( i = minprot; i != NIL; i = protnext[i] )
				{
				d = tbldiff( state, i, extrct[1 - extptr] );
				if ( d < mindiff )
					{
					extptr = 1 - extptr;
					mindiff = d;
					minprot = i;
					}
				}
			}

		/* Check if the proto we've decided on as our best bet is close
		 * enough to the state we want to match to be usable.
		 */

		if ( mindiff * 100 > totaltrans * ACCEPTABLE_DIFF_PERCENTAGE )
			{
			/* No good.  If the state is homogeneous enough,
			 * we make a template out of it.  Otherwise, we
			 * make a proto.
			 */

			if ( comfreq * 100 >=
			     totaltrans * TEMPLATE_SAME_PERCENTAGE )
				mktemplate( state, statenum, comstate );

			else
				{
				mkprot( state, statenum, comstate );
				mkentry( state, numecs, statenum,
					JAMSTATE, totaltrans );
				}
			}

		else
			{ /* use the proto */
			mkentry( extrct[extptr], numecs, statenum,
				prottbl[minprot], mindiff );

			/* If this state was sufficiently different from the
			 * proto we built it from, make it, too, a proto.
			 */

			if ( mindiff * 100 >=
			     totaltrans * NEW_PROTO_DIFF_PERCENTAGE )
				mkprot( state, statenum, comstate );

			/* Since mkprot added a new proto to the proto queue,
			 * it's possible that "minprot" is no longer on the
			 * proto queue (if it happened to have been the last
			 * entry, it would have been bumped off).  If it's
			 * not there, then the new proto took its physical
			 * place (though logically the new proto is at the
			 * beginning of the queue), so in that case the
			 * following call will do nothing.
			 */

			mv2front( minprot );
			}
		}
	}



void cmptmps()
	{
	int tmpstorage[CSIZE + 1];
	register int *tmp = tmpstorage, i, j;
	int totaltrans, trans;

	peakpairs = numtemps * numecs + tblend;

	if ( usemecs )
		{
		/* Create equivalence classes based on data gathered on
		 * template transitions.
		 */
		nummecs = cre8ecs( tecfwd, tecbck, numecs );
		}

	else
		nummecs = numecs;

	while ( lastdfa + numtemps + 1 >= current_max_dfas )
		increase_max_dfas();

	/* Loop through each template. */

	for ( i = 1; i <= numtemps; ++i )
		{
		/* Number of non-jam transitions out of this template. */
		totaltrans = 0;

		for ( j = 1; j <= numecs; ++j )
			{
			trans = tnxt[numecs * i + j];

			if ( usemecs )
				{
				/* The absolute value of tecbck is the
				 * meta-equivalence class of a given
				 * equivalence class, as set up by cre8ecs().
				 */
				if ( tecbck[j] > 0 )
					{
					tmp[tecbck[j]] = trans;

					if ( trans > 0 )
						++totaltrans;
					}
				}

			else
				{
				tmp[j] = trans;

				if ( trans > 0 )
					++totaltrans;
				}
			}

		/* It is assumed (in a rather subtle way) in the skeleton
		 * that if we're using meta-equivalence classes, the def[]
		 * entry for all templates is the jam template, i.e.,
		 * templates never default to other non-jam table entries
		 * (e.g., another template)
		 */

		/* Leave room for the jam-state after the last real state. */
		mkentry( tmp, nummecs, lastdfa + i + 1, JAMSTATE, totaltrans );
		}
	}



/* expand_nxt_chk - expand the next check arrays */

void expand_nxt_chk()
	{
	register int old_max = current_max_xpairs;

	current_max_xpairs += MAX_XPAIRS_INCREMENT;

	++num_reallocs;

	nxt = reallocate_integer_array( nxt, current_max_xpairs );
	chk = reallocate_integer_array( chk, current_max_xpairs );

	zero_out( (char *) (chk + old_max),
		(size_t) (MAX_XPAIRS_INCREMENT * sizeof( int )) );
	}



int find_table_space( state, numtrans )
int *state, numtrans;
	{
	/* Firstfree is the position of the first possible occurrence of two
	 * consecutive unused records in the chk and nxt arrays.
	 */
	register int i;
	register int *state_ptr, *chk_ptr;
	register int *ptr_to_last_entry_in_state;

	/* If there are too many out-transitions, put the state at the end of
	 * nxt and chk.
	 */
	if ( numtrans > MAX_XTIONS_FULL_INTERIOR_FIT )
		{
		/* If table is empty, return the first available spot in
		 * chk/nxt, which should be 1.
		 */
		if ( tblend < 2 )
			return 1;

		/* Start searching for table space near the end of
		 * chk/nxt arrays.
		 */
		i = tblend - numecs;
		}

	else
		/* Start searching for table space from the beginning
		 * (skipping only the elements which will definitely not
		 * hold the new state).
		 */
		i = firstfree;

	while ( 1 )	/* loops until a space is found */
		{
		while ( i + numecs >= current_max_xpairs )
			expand_nxt_chk();

		/* Loops until space for end-of-buffer and action number
		 * are found.
		 */
		while ( 1 )
			{
			/* Check for action number space. */
			if ( chk[i - 1] == 0 )
				{
				/* Check for end-of-buffer space. */
				if ( chk[i] == 0 )
					break;

				else
					/* Since i != 0, there is no use
					 * checking to see if (++i) - 1 == 0,
					 * because that's the same as i == 0,
					 * so we skip a space.
					 */
					i += 2;
				}

			else
				++i;

			while ( i + numecs >= current_max_xpairs )
				expand_nxt_chk();
			}

		/* If we started search from the beginning, store the new
		 * firstfree for the next call of find_table_space().
		 */
		if ( numtrans <= MAX_XTIONS_FULL_INTERIOR_FIT )
			firstfree = i + 1;

		/* Check to see if all elements in chk (and therefore nxt)
		 * that are needed for the new state have not yet been taken.
		 */

		state_ptr = &state[1];
		ptr_to_last_entry_in_state = &chk[i + numecs + 1];

		for ( chk_ptr = &chk[i + 1];
		      chk_ptr != ptr_to_last_entry_in_state; ++chk_ptr )
			if ( *(state_ptr++) != 0 && *chk_ptr != 0 )
				break;

		if ( chk_ptr == ptr_to_last_entry_in_state )
			return i;

		else
		++i;
		}
	}


void inittbl()
	{
	register int i;

	zero_out( (char *) chk, (size_t) (current_max_xpairs * sizeof( int )) );

	tblend = 0;
	firstfree = tblend + 1;
	numtemps = 0;

	if ( usemecs )
		{
		/* Set up doubly-linked meta-equivalence classes; these
		 * are sets of equivalence classes which all have identical
		 * transitions out of TEMPLATES.
		 */

		tecbck[1] = NIL;

		for ( i = 2; i <= numecs; ++i )
			{
			tecbck[i] = i - 1;
			tecfwd[i - 1] = i;
			}

		tecfwd[numecs] = NIL;
		}
	}


/* mkdeftbl - make the default, "jam" table entries */

void mkdeftbl()
	{
	int i;

	jamstate = lastdfa + 1;

	++tblend; /* room for transition on end-of-buffer character */

	while ( tblend + numecs >= current_max_xpairs )
		expand_nxt_chk();

	/* Add in default end-of-buffer transition. */
	nxt[tblend] = end_of_buffer_state;
	chk[tblend] = jamstate;

	for ( i = 1; i <= numecs; ++i )
		{
		nxt[tblend + i] = 0;
		chk[tblend + i] = jamstate;
		}

	jambase = tblend;

	base[jamstate] = jambase;
	def[jamstate] = 0;

	tblend += numecs;
	++numtemps;
	}



void mkentry( state, numchars, statenum, deflink, totaltrans )
register int *state;
int numchars, statenum, deflink, totaltrans;
	{
	register int minec, maxec, i, baseaddr;
	int tblbase, tbllast;

	if ( totaltrans == 0 )
		{ /* there are no out-transitions */
		if ( deflink == JAMSTATE )
			base[statenum] = JAMSTATE;
		else
			base[statenum] = 0;

		def[statenum] = deflink;
		return;
		}

	for ( minec = 1; minec <= numchars; ++minec )
		{
		if ( state[minec] != SAME_TRANS )
			if ( state[minec] != 0 || deflink != JAMSTATE )
				break;
		}

	if ( totaltrans == 1 )
		{
		/* There's only one out-transition.  Save it for later to fill
		 * in holes in the tables.
		 */
		stack1( statenum, minec, state[minec], deflink );
		return;
		}

	for ( maxec = numchars; maxec > 0; --maxec )
		{
		if ( state[maxec] != SAME_TRANS )
			if ( state[maxec] != 0 || deflink != JAMSTATE )
				break;
		}

	/* Whether we try to fit the state table in the middle of the table
	 * entries we have already generated, or if we just take the state
	 * table at the end of the nxt/chk tables, we must make sure that we
	 * have a valid base address (i.e., non-negative).  Note that
	 * negative base addresses dangerous at run-time (because indexing
	 * the nxt array with one and a low-valued character will access
	 * memory before the start of the array.
	 */

	/* Find the first transition of state that we need to worry about. */
	if ( totaltrans * 100 <= numchars * INTERIOR_FIT_PERCENTAGE )
		{
		/* Attempt to squeeze it into the middle of the tables. */
		baseaddr = firstfree;

		while ( baseaddr < minec )
			{
			/* Using baseaddr would result in a negative base
			 * address below; find the next free slot.
			 */
			for ( ++baseaddr; chk[baseaddr] != 0; ++baseaddr )
				;
			}

		while ( baseaddr + maxec - minec + 1 >= current_max_xpairs )
			expand_nxt_chk();

		for ( i = minec; i <= maxec; ++i )
			if ( state[i] != SAME_TRANS &&
			     (state[i] != 0 || deflink != JAMSTATE) &&
			     chk[baseaddr + i - minec] != 0 )
				{ /* baseaddr unsuitable - find another */
				for ( ++baseaddr;
				      baseaddr < current_max_xpairs &&
				      chk[baseaddr] != 0; ++baseaddr )
					;

				while ( baseaddr + maxec - minec + 1 >=
					current_max_xpairs )
					expand_nxt_chk();

				/* Reset the loop counter so we'll start all
				 * over again next time it's incremented.
				 */

				i = minec - 1;
				}
		}

	else
		{
		/* Ensure that the base address we eventually generate is
		 * non-negative.
		 */
		baseaddr = MAX( tblend + 1, minec );
		}

	tblbase = baseaddr - minec;
	tbllast = tblbase + maxec;

	while ( tbllast + 1 >= current_max_xpairs )
		expand_nxt_chk();

	base[statenum] = tblbase;
	def[statenum] = deflink;

	for ( i = minec; i <= maxec; ++i )
		if ( state[i] != SAME_TRANS )
			if ( state[i] != 0 || deflink != JAMSTATE )
				{
				nxt[tblbase + i] = state[i];
				chk[tblbase + i] = statenum;
				}

	if ( baseaddr == firstfree )
		/* Find next free slot in tables. */
		for ( ++firstfree; chk[firstfree] != 0; ++firstfree )
			;

	tblend = MAX( tblend, tbllast );
	}



void mk1tbl( state, sym, onenxt, onedef )
int state, sym, onenxt, onedef;
	{
	if ( firstfree < sym )
		firstfree = sym;

	while ( chk[firstfree] != 0 )
		if ( ++firstfree >= current_max_xpairs )
			expand_nxt_chk();

	base[state] = firstfree - sym;
	def[state] = onedef;
	chk[firstfree] = state;
	nxt[firstfree] = onenxt;

	if ( firstfree > tblend )
		{
		tblend = firstfree++;

		if ( firstfree >= current_max_xpairs )
			expand_nxt_chk();
		}
	}


/* mkprot - create new proto entry */

void mkprot( state, statenum, comstate )
int state[], statenum, comstate;
	{
	int i, slot, tblbase;

	if ( ++numprots >= MSP || numecs * numprots >= PROT_SAVE_SIZE )
		{
		/* Gotta make room for the new proto by dropping last entry in
		 * the queue.
		 */
		slot = lastprot;
		lastprot = protprev[lastprot];
		protnext[lastprot] = NIL;
		}

	else
		slot = numprots;

	protnext[slot] = firstprot;

	if ( firstprot != NIL )
		protprev[firstprot] = slot;

	firstprot = slot;
	prottbl[slot] = statenum;
	protcomst[slot] = comstate;

	/* Copy state into save area so it can be compared with rapidly. */
	tblbase = numecs * (slot - 1);

	for ( i = 1; i <= numecs; ++i )
		protsave[tblbase + i] = state[i];
	}



void mktemplate( state, statenum, comstate )
int state[], statenum, comstate;
	{
	int i, numdiff, tmpbase, tmp[CSIZE + 1];
	Char transset[CSIZE + 1];
	int tsptr;

	++numtemps;

	tsptr = 0;

	/* Calculate where we will temporarily store the transition table
	 * of the template in the tnxt[] array.  The final transition table
	 * gets created by cmptmps().
	 */

	tmpbase = numtemps * numecs;

	if ( tmpbase + numecs >= current_max_template_xpairs )
		{
		current_max_template_xpairs += MAX_TEMPLATE_XPAIRS_INCREMENT;

		++num_reallocs;

		tnxt = reallocate_integer_array( tnxt,
			current_max_template_xpairs );
		}

	for ( i = 1; i <= numecs; ++i )
		if ( state[i] == 0 )
			tnxt[tmpbase + i] = 0;
		else
			{
			transset[tsptr++] = i;
			tnxt[tmpbase + i] = comstate;
			}

	if ( usemecs )
		mkeccl( transset, tsptr, tecfwd, tecbck, numecs, 0 );

	mkprot( tnxt + tmpbase, -numtemps, comstate );

	/* We rely on the fact that mkprot adds things to the beginning
	 * of the proto queue.
	 */

	numdiff = tbldiff( state, firstprot, tmp );
	mkentry( tmp, numecs, statenum, -numtemps, numdiff );
	}


/* mv2front - move proto queue element to front of queue */

void mv2front( qelm )
int qelm;
	{
	if ( firstprot != qelm )
		{
		if ( qelm == lastprot )
			lastprot = protprev[lastprot];

		protnext[protprev[qelm]] = protnext[qelm];

		if ( protnext[qelm] != NIL )
			protprev[protnext[qelm]] = protprev[qelm];

		protprev[qelm] = NIL;
		protnext[qelm] = firstprot;
		protprev[firstprot] = qelm;
		firstprot = qelm;
		}
	}



void place_state( state, statenum, transnum )
int *state, statenum, transnum;
	{
	register int i;
	register int *state_ptr;
	int position = find_table_space( state, transnum );

	/* "base" is the table of start positions. */
	base[statenum] = position;

	/* Put in action number marker; this non-zero number makes sure that
	 * find_table_space() knows that this position in chk/nxt is taken
	 * and should not be used for another accepting number in another
	 * state.
	 */
	chk[position - 1] = 1;

	/* Put in end-of-buffer marker; this is for the same purposes as
	 * above.
	 */
	chk[position] = 1;

	/* Place the state into chk and nxt. */
	state_ptr = &state[1];

	for ( i = 1; i <= numecs; ++i, ++state_ptr )
		if ( *state_ptr != 0 )
			{
			chk[position + i] = i;
			nxt[position + i] = *state_ptr;
			}

	if ( position + numecs > tblend )
		tblend = position + numecs;
	}



void stack1( statenum, sym, nextstate, deflink )
int statenum, sym, nextstate, deflink;
	{
	if ( onesp >= ONE_STACK_SIZE - 1 )
		mk1tbl( statenum, sym, nextstate, deflink );

	else
		{
		++onesp;
		onestate[onesp] = statenum;
		onesym[onesp] = sym;
		onenext[onesp] = nextstate;
		onedef[onesp] = deflink;
		}
	}



int tbldiff( state, pr, ext )
int state[], pr, ext[];
	{
	register int i, *sp = state, *ep = ext, *protp;
	register int numdiff = 0;

	protp = &protsave[numecs * (pr - 1)];

	for ( i = numecs; i > 0; --i )
		{
		if ( *++protp == *++sp )
			*++ep = SAME_TRANS;
		else
			{
			*++ep = *sp;
			++numdiff;
			}
		}

	return numdiff;
	}
