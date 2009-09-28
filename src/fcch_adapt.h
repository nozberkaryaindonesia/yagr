/*
 * fcch_adapt.h
 *
 * Header for an adaptive FCCH sync detector based on this paper :
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

#ifndef __GSM_FCCH_ADAPT_H__
#define __GSM_FCCH_ADAPT_H__


#include "fcch.h"


class FCCHAdaptDetector : public FCCHDetector
{
public:
	/* Subclass encapsulating the options */
	class Params
	{
	public:
		Params(void) :
			af_D(8),
			af_k(17),
			bd_p(0.05),
			bd_le_th(0.15),
			bd_le_minlen(100),
			bed_hlen(5),
			bed_p(0.03),
			bed_he_th(6),
			bed_he_minlen(3) {}

		/* Adaptive filter predictor */
		int af_D;		/* Prediction delay */
		int af_k;		/* Filter length */

		/* Burst detection */
		float bd_p;		/* Forgetting factor */
		float bd_le_th;		/* 'low-error' threshold */
		float bd_le_minlen;	/* # 'low-error' to match burst */

		/* Burst End detection */
		int   bed_hlen;		/* Error tracking history length */
		float bed_p;		/* Forgetting factor */
		float bed_he_th;	/* 'high-error' threshold (as stddev factor) */
		float bed_he_minlen;	/* # 'high-error' to match burst end */
	};

	/* Constructor / Destructor */
	FCCHAdaptDetector(void);
	FCCHAdaptDetector(const Params &p);
	virtual ~FCCHAdaptDetector(void);

	/* Methods */
	void reset(void);
	int find_next(complex *samples, int n_samples, int *fcch_end_pos, float *ang_freq);

private:
	/* Private methods */
	void _do_init(void);

	int _search_next_burst(float e);
	int  _search_burst_end(float e);
	void _switch_to_search_next_burst(void);
	void _switch_to_search_burst_end(void);
	
	float _next_error(complex *x);

	float _acc_phase_diff(complex *x, int len);
	void _reset_phase_diff_avg(void);
	void _update_phase_diff_avg(complex *x, int len);
	float _finish_phase_diff_avg(void);

	/* Parameters */
	const Params p;

	/* General state */
	enum {
		SEARCH_NEXT_BURST,
		SEARCH_BURST_END,
	} state;

	/* Adaptive Filter predictor */
	complex *s_af_w;		/* Filter coeffs */

	/* Burst Detection */
	float s_bd_e_mavg;		/* Error moving average */
	int s_bd_le_cnt;		/* # of samples with 'low-error' */

	/* Burst End Detection */
	float s_bed_he_cnt;		/* # of samples with 'high-error' */
	float *s_bed_e_mavg;		/* Error moving average history (0=oldest) */
	float *s_bed_e_msd2;		/* Error moving stddev^2 history (0=oldest) */

	/* Current burst */
	int s_burst_len;		/* Current burst length */
	float s_burst_pdacc;		/* Current burst avg phase diff */
};


#endif /* __GSM_FCCH_ADAPT_H__ */

