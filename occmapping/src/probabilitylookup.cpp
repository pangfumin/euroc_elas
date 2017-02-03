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

#include "probabilitylookup.h"
#include <cmath>
#include <vector>

namespace occmapping {
	using namespace std;

	ProbabilityLookup::ProbabilityLookup(double ocTreeResolution, double depthErrorResolution, double depthErrorInterval,
		double maxDepth, double probMin, double probMax, double cutOffLimit, double depthErrorScale)
		:depthErrorInterval(depthErrorInterval) {
		
		vector<float> negativeRange, positiveRange;
		
		for(unsigned int zScaled = 1;; zScaled++) {
			// Get discretized sample for z
			double z = zScaled*depthErrorInterval;
			
			if(z > maxDepth)
				break;
		
			// Standard deviation for depth measurements at depth z
			double sigma = depthErrorScale*z*z;
			
			// Compute probabilities for points with smaller depth (negative range)
			negativeRange.clear();
			for(int i=0;; i--) {
				double prob = computeCDF(i*depthErrorResolution, 0, sigma);
				negativeRange.push_back((float)(prob * (probMax - probMin) + probMin));
				if(prob < cutOffLimit)
					break;
			}
			
			// Compute probabilities for points with larger depth (positive range)
			positiveRange.clear();
			for(unsigned int i=1;i<negativeRange.size(); i++) {
				double prob = computeCDF(i*depthErrorResolution, 0, sigma);
				positiveRange.push_back((float)(prob * (probMax - probMin) + probMin));
			}
			
			// Create new entry of the lookup table
			lookupTable.push_back(vector<float>(negativeRange.size() + positiveRange.size()));
			copy(negativeRange.rbegin(), negativeRange.rend(), lookupTable.back().begin());
			copy(positiveRange.begin(), positiveRange.end(), lookupTable.back().begin() + negativeRange.size());
		}
	}

	double ProbabilityLookup::computeCDF(double x, double mean, double sigma) {
		// The cumulative distribution function for the normal distribution
		return 0.5 * (1.0 + erf((x - mean) / (sigma * M_SQRT2)));
	}
}