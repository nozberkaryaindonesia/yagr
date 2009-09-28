/*
 * fcch_simple.h
 *
 * Header for a simple (but effective !) FCCH sync detector
 *
 * The algorithm itself if heavily based on the one used by
 * Piotr Krysik <perper@o2.pl> in gsm-receiver.
 *
 *
 * Copyright (C) 2009 Sylvain Munaut <tnt@246tNt.com>
 *
 * This file is part of 'yagr'.
 *
 * yagr is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * yagr is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with yagr.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __GSM_FCCH_SIMPLE_H__
#define __GSM_FCCH_SIMPLE_H__


#include "fcch.h"


class FCCHSimpleDetector : public FCCHDetector
{
public:
	/* Subclass encapsulating the options */
	class Params
	{
	public:
		Params(int osr=4) :
			hits_needed(142 * osr),	/* FIXME constant */
			misses_limit(1 * osr) {}

		int hits_needed;
		int misses_limit;
	};

	/* Constructor / Destructor */
	FCCHSimpleDetector(void);
	FCCHSimpleDetector(const Params &p);
	virtual ~FCCHSimpleDetector(void);

	/* Methods */
	void reset(void);
	int find_next(complex *samples, int n_samples, int *fcch_end_pos, float *ang_freq);

private:
	/* Parameters */
	const Params p;

	/* General state */
	enum {
		SEARCH,
		FOUND_SOMETHING,
	} state;

	/* Counters */
	int hits_count;
	int misses_count;
	int trail_misses_count;

	/* Phase diff accumulator (hits_count samples) */
	float pd_acc;
};


#endif /* __GSM_FCCH_SIMPLE_H__ */

