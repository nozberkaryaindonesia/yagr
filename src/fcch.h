/*
 * fcch.h
 *
 * Interface for a FCCH sync detector
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

#ifndef __GSM_FCCH_H__
#define __GSM_FCCH_H__


#include "types.h"


class FCCHDetector
{
public:
	virtual ~FCCHDetector(void) { };

	virtual void reset(void) = 0;
	virtual int find_next(
		complex *samples, int n_samples,
		int *fcch_end_post, float *ang_freq) = 0;
};


#endif /* __GSM_FCCH_H__ */

