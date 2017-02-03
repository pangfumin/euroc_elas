/*
 * Copyright (c) 2013-2014, Konstantin Schauwecker
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PROBABILITYNLOOKUP_H
#define PROBABILITYLOOKUP_H

#include <cstdio>
#include <vector>

namespace occmapping {
	// Lookup table for the probability distribution of
	// endpoint observations in the occupancy map
	class ProbabilityLookup {
	public:
		ProbabilityLookup(double ocTreeResolution, double depthErrorResolution, double depthErrorInterval,
			double maxDepth, double probMin, double probMax, double cutOffLimit, double depthErrorScale);
		
		// Performs a lookup in the precomputed probability table for depth z.
		// Return value is a list of values for the probability that a point 
		// at z + i*depthErrorResolution is inside the obstacle detected at depth z.
		// The range of i is symmetric in positive / negative direction. Hence,
		// for the middle element i=0.
		std::vector<float>* lookupEntry(double z) {
			int zScaled = (int)(z / depthErrorInterval + 0.5);
			if(zScaled >= (int)lookupTable.size() || zScaled < 0)
				return NULL;
			else return &lookupTable[zScaled];
		}
		
	private:
		double depthErrorInterval;
		std::vector<std::vector<float> > lookupTable;
		
		double computeCDF(double x, double mean, double sigma);
		double computeRangeProbability(double mean, double sigma, double minRange, double maxRange);
	};
}
#endif
