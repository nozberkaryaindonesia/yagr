/*
 * fcch_simple.cc
 *
 * Implementation  of a simple (but effective !) FCCH sync detector
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

#include "fcch_simple.h"


FCCHSimpleDetector::FCCHSimpleDetector(void)
{
	this->reset();
}

FCCHSimpleDetector::FCCHSimpleDetector(const Params &p) :
		p(p)
{
	this->reset();
}

FCCHSimpleDetector::~FCCHSimpleDetector(void)
{
	/* nothing to do ... */
}


void FCCHSimpleDetector::reset(void)
{
	this->state = SEARCH;
	this->hits_count = 0;
	this->misses_count = 0;
	this->trail_misses_count = 0;
	this->pd_acc = 0.0f;
}

static inline float
phase_diff(complex s0, complex s1)
{
	return std::arg(s1 * std::conj(s0));
}

int FCCHSimpleDetector::find_next(complex *samples, int n_samples, int *fcch_end_pos, float *ang_freq)
{
	int i;

	for (i=0; i<n_samples-1; i++) {

		float pd = phase_diff(samples[i], samples[i+1]);

		switch (this->state) {
			case SEARCH:
				if (pd > 0)
					this->state = FOUND_SOMETHING;
				break;

			case FOUND_SOMETHING:
				if (pd > 0) {
					/* Adjust counters */
					this->hits_count++;
					this->trail_misses_count = 0;

					/* Accumulate the value */
					this->pd_acc += pd;
				} else {
					/* Adjust counters */
					this->misses_count++;
					this->trail_misses_count++;

					/* Are we done ? */
					if (this->misses_count > this->p.misses_limit) {
						if (this->hits_count >= this->p.hits_needed) {
							if (*ang_freq)
								*ang_freq = this->pd_acc / this->hits_count;

							*fcch_end_pos = i + 1 - this->trail_misses_count;
							this->reset();
							return i;
						} else {
							this->reset();
						}
					}
				}
				break;
		}
	}

	/* We ran out of sample */
	*fcch_end_pos = -1;
	return i;
}

