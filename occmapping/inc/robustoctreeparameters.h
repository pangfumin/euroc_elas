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

#ifndef ROBUSTOCTREEPARAMETERS_H
#define ROBUSTOCTREEPARAMETERS_H

namespace occmapping {
	// Data structure for storing parameters neccessary for
	// occupancy mapping. You can use the default constructor
	// to obtain some reasonamble default parameters.
	struct RobustOcTreeParameters {
		// Size of a voxel in the ocTree.
		double octreeResolution;
		
		// Points with a z-coordinates below the minimum height will not be mapped.
		double minHeight;
		
		// Points with a z-coordinates above the minimum height will not be mapped.
		double maxHeight;
		
		// Maximum distance up to which points will be mapped.
		double maxRange;
		
		// P(H | V, O): Probability of measuring a hit if a voxel is
		// vissible and occupied.
		float probHitIfOccupied;
		
		// P(H | V, !O): Probability of measuring a hit if a voxel is
		// visible but not occupied.
		float probHitIfNotOccupied;
		
		// P(H | !V): Probability of a measuring a hit if a voxel is not visible.
		float probHitIfNotVisible;
		
		// P(V_vi | C_vi, V_vi-1): Probability of a voxel being visible if it
		// is locally occluded and the previous voxel was visible.
		float probVisibleIfOccluded;
		
		// P(V_vi | !C_vi, V_vi-1): Probability of a voxel being visible if it
		// is not locally occluded and the previous voxel was visible.
		float probVisibleIfNotOccluded;
		
		// Resolution (as fraction of voxel size) with with to precompute the depth error.
		double depthErrorResolution;
		
		// Interval (as fraction of voxel size) at which to precompute the depth error.
		double depthErrorInterval;
		
		// Minimum occupancy probability for a voxel to be considered occupied.
		float occupancyProbThreshold;
		
		// Maximum clamping threshold for occupancy probability.
		float clampingThresholdMax;
		
		// Minimum clamping threshold for occupancy probability.
		float clampingThresholdMin;
		
		// Maximum clamping threshold for visibility probability.
		float visibilityClampingMax;
		
		// Mainimum clamping threshold for visibility probability.
		float visibilityClampingMin;
		
		// Scaling factor for quadratic increase of depth error.
		// For a stereo camera this is
		// s / (b * f),
		// where s: disparity standard deviation
		//       b: baseline
		//       f: focal length in pixel
		double depthErrorScale;
		
		// Constructor that sets reasonable default parameters
		RobustOcTreeParameters():
			octreeResolution(0.2),
			minHeight(-1e10),
			maxHeight(1e10),
			maxRange(10),
			probHitIfOccupied(0.55),
			probHitIfNotOccupied(0.43),
			probHitIfNotVisible(0.05),
			probVisibleIfOccluded(0.2),
			probVisibleIfNotOccluded(1.0),
			depthErrorResolution(0.05),
			depthErrorInterval(0.1),
			occupancyProbThreshold(0.5),
			clampingThresholdMax(0.971 /* 3.5 in log odds */),
			clampingThresholdMin(0.1192 /* -2 in log odds */),
			visibilityClampingMax(0.7),
			visibilityClampingMin(0.1),
			depthErrorScale(0.0028 /*depends on stereo camera*/) {
		}		
	};
}

#endif
