/*
 * fcch_adapt.cc
 *
 * Implementation of an adaptive FCCH sync detector based on this paper :
 * "Robust frequency burst detection algorithm for GSM/GPRS"
 * by NARENDRA VARMA G. , SAHU Usha & PRABHU CHARAN G.
 *
 * The exact method to find the burst _end_ is not described in the
 * paper tough and I just made it up.
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

#include "fcch_adapt.h"

#include <stdio.h>

FCCHAdaptDetector::FCCHAdaptDetector(void)
{
	this->_do_init();
}

FCCHAdaptDetector::FCCHAdaptDetector(const Params &p) :
		p(p)
{
	this->_do_init();
}

FCCHAdaptDetector::~FCCHAdaptDetector(void)
{
	/* Free up resources */
	delete[] this->s_af_w;
	delete[] this->s_bed_e_mavg;
	delete[] this->s_bed_e_msd2;
}


void FCCHAdaptDetector::reset(void)
{
	/* Clear filter */
	for (int i=0; i<this->p.af_k; i++)
		this->s_af_w[i] = complex(0.0, 0.0);

	/* Burst Detect error moving avg is tracked all the time */
	this->s_bd_e_mavg = 1.0;

	/* Phase diff acc */
	this->_reset_phase_diff_avg();

	/* Start search */
	this->_switch_to_search_next_burst();
}

#define moving_seq_upd(v,nv,p) do { v = ((1.0f - (p)) * (v)) + ((p) * (nv)); } while (0)

int FCCHAdaptDetector::find_next(complex *samples, int n_samples, int *fcch_end_pos, float *ang_freq)
{
	int i, fcch_start = 0;
	int adv = this->p.af_D + this->p.af_k - 1;

	for (i=0; i<n_samples-adv; i++) {
		float e;

		/* Get the next error */
		e = this->_next_error(&samples[i]);

		/* Continuous error avg tracking */
		moving_seq_upd(this->s_bd_e_mavg, e, this->p.bd_p);

		/* State dependent processing */
		switch (this->state) {
			case SEARCH_NEXT_BURST:
				if (this->_search_next_burst(e)) {
					/* Update recording phase diff */
					/* Note that since the filter adapted,
					 * the burst started _before_ but no
					 * need to go back too far */
					this->_update_phase_diff_avg(
						&samples[i-this->s_bd_le_cnt+1],
						this->s_bd_le_cnt
					);

					/* Record this pos */
					fcch_start = i + 1;
				}
				break;

			case SEARCH_BURST_END:
				if (this->_search_burst_end(e)) {
					/* Return exact position */
					*fcch_end_pos = i + adv - this->s_bed_he_cnt + 1;

					/* Update phase diff average */
					/* We could go up to *fcch_end_post but
					 * the end might not be precise ... */
					this->_update_phase_diff_avg(
						&samples[fcch_start],
						i - fcch_start
					);

					/* Get angular frequency if needed */
					if (ang_freq)
						*ang_freq = this->_finish_phase_diff_avg();

					/* Reset it */
					this->_reset_phase_diff_avg();

					/* Consumed samples (for a later resume) */
					return i + 1;
				}
				break;
		}
	}

	/* Possibly need to record phase diff history */
	switch (this->state) {
		case SEARCH_NEXT_BURST:
			/* Acc the phase diff we have so far in case we were
			 * interrupted inside FCCH */
			this->_update_phase_diff_avg(
				&samples[i-this->s_bd_le_cnt],
				this->s_bd_le_cnt
			);
			break;

		case SEARCH_BURST_END:
			/* Acc the phase diff since we were _in_ FCCH for sure */
			this->_update_phase_diff_avg(
				&samples[fcch_start],
				i - fcch_start
			);
			break;
	}

	/* Nothing found in the sample we had */
	*fcch_end_pos = -1;
	return i;
}

void FCCHAdaptDetector::_do_init(void)
{
	/* Create the arrays */
	this->s_af_w = new complex[this->p.af_k];
	this->s_bed_e_mavg = new float[this->p.bed_hlen];
	this->s_bed_e_msd2 = new float[this->p.bed_hlen];

	/* Reset */
	this->reset();
}

inline int FCCHAdaptDetector::_search_next_burst(float e)
{
	/* Detect error < bd_le_th longer than bd_le_minlen */
	if (this->s_bd_e_mavg < this->p.bd_le_th) {
		if (++this->s_bd_le_cnt >= this->p.bd_le_minlen) {
			/* We're in a burst, seek the end ! */
			this->_switch_to_search_burst_end();

			return 1;
		}
	} else {
		this->s_bd_le_cnt = 0;
		this->_reset_phase_diff_avg(); /* Clear the left overs */
	}

	return 0;
}

inline int FCCHAdaptDetector::_search_burst_end(float e)
{
	/* Did we hit the end */
	float lmt = this->s_bed_e_mavg[0] +
		this->p.bed_he_th * sqrtf(this->s_bed_e_msd2[0]);

	if (e > lmt) {
		if (++this->s_bed_he_cnt >= this->p.bed_he_minlen) {
			/* Switch mode back to search burst */
			this->_switch_to_search_next_burst();

			return 1;
		}
	} else {
		this->s_bed_he_cnt = 0;
	}

	/* Update error tracking history */
	int li = this->p.bed_hlen-1;
	for (int j=0; j<li; j++) {
		this->s_bed_e_mavg[j] = this->s_bed_e_mavg[j+1];
		this->s_bed_e_msd2[j] = this->s_bed_e_msd2[j+1];
	}
	float d = e - this->s_bed_e_mavg[li];
	moving_seq_upd(this->s_bed_e_mavg[li], e, this->p.bed_p);
	moving_seq_upd(this->s_bed_e_msd2[li], d * d, this->p.bed_p);

	return 0;
}

inline void FCCHAdaptDetector::_switch_to_search_next_burst(void)
{
	/* Reset search counter */
	this->s_bd_le_cnt = 0;

	/* Switch state */
	this->state = SEARCH_NEXT_BURST;
}

inline void FCCHAdaptDetector::_switch_to_search_burst_end(void)
{
	/* Reset search counter */
	this->s_bed_he_cnt = 0;

	/* Pre-init error tracking history */
	for (int i=0; i<this->p.bed_hlen; i++) {
		float e = this->s_bd_e_mavg;
		this->s_bed_e_mavg[i] = e;
		this->s_bed_e_msd2[i] = e * e;
	}

	/* Switch state */
	this->state = SEARCH_BURST_END;
}

/*
 * This function assumes there are sufficient samples !
 */
inline float FCCHAdaptDetector::_next_error(complex *x)
{
	int i, k, km1, D;
	float E_t, E, G;
	complex *w, e, y;

	/* Get params & state */
	k = this->p.af_k;
	km1 = k - 1;
	D = this->p.af_D;
	w = this->s_af_w;

	/* Input energy */
	E_t = 0.0;
	for (i=0; i<k; i++)
		E_t += std::norm(x[i]);

	E = E_t / k;

	/* Get the convergence factor */
	G = 0.5 / (E_t + 0.01);

	/* Predictor ouput */
	y = complex(0.0, 0.0);
	for (i=0; i<k; i++)	/* FIXME filter apply function ? */
		y += w[i] * x[km1-i];

	/* Error */
	e = x[km1+D] - y;

	/* Filter update */
	for (i=0; i<k; i++)
		w[i] += G * e * std::conj(x[km1-i]);

	/* Return normalized error energy */
	return std::norm(e) / E;
}

float FCCHAdaptDetector::_acc_phase_diff(complex *x, int len)
{
	int i;
	float pdacc = 0.0f;
	for (i=0; i<len-1; i++)	/* FIXME phase_diff function ? */
		pdacc += std::arg(x[i] * std::conj(x[i+1]));
	return pdacc;
}

void FCCHAdaptDetector::_reset_phase_diff_avg(void)
{
	this->s_burst_len = 0;
	this->s_burst_pdacc = 0.0f;
}

void FCCHAdaptDetector::_update_phase_diff_avg(complex *s, int len)
{
	this->s_burst_len += len;
	this->s_burst_pdacc += this->_acc_phase_diff(s, len);;
}

float FCCHAdaptDetector::_finish_phase_diff_avg(void)
{
	return this->s_burst_pdacc / this->s_burst_len;
}

